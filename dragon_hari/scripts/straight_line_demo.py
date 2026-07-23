#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class StraightLineDemo:
    SQUARE_PUBLISH_DURATION_SEC = 2.0
    SQUARE_PUBLISH_RATE_HZ = 10.0

    YAW_ALIGNMENT_TOLERANCE_RAD = math.radians(5.0)
    YAW_ALIGNMENT_TIMEOUT_SEC = 20.0
    YAW_ALIGNMENT_CHECK_RATE_HZ = 20.0


    JOINT_NAMES = (
        "joint1_pitch",
        "joint1_yaw",
        "joint2_pitch",
        "joint2_yaw",
        "joint3_pitch",
        "joint3_yaw",
    )

    SQUARE_POSITIONS = (
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
    )

    V14_YAW_OFFSET = 3.0 * math.pi / 4.0

    def __init__(self, demo_mode, goal_world_x, goal_world_y):
        self.demo_mode = demo_mode
        self.goal_world_xy = (goal_world_x, goal_world_y)

        self.cog_x = None
        self.cog_y = None
        self.cog_z = None
        self.cog_yaw = None
        self.baselink_yaw = None
        self.vertex_forward_target_baselink_yaw = None

        self.joint_positions = {}
        self.joint_state_rx_time = None

        self.joint_command_pub = rospy.Publisher(
            "/dragon/joints_ctrl",
            JointState,
            queue_size=10,
        )

        self.navigation_goal_pub = rospy.Publisher(
            "/dragon/target_pose",
            PoseStamped,
            queue_size=1,
        )

        self.joint_state_sub = rospy.Subscriber(
            "/dragon/joint_states",
            JointState,
            self.joint_state_callback,
            queue_size=1,
        )


        self.cog_odom_sub = rospy.Subscriber(
            "/dragon/uav/cog/odom",
            Odometry,
            self.cog_odom_callback,
            queue_size=1,
        )

        self.baselink_odom_sub = rospy.Subscriber(
            "/dragon/uav/baselink/odom",
            Odometry,
            self.baselink_odom_callback,
            queue_size=1,
        )
    
    def joint_state_callback(self, msg):
        positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
            if name in self.JOINT_NAMES and math.isfinite(position)
        }

        self.joint_positions = positions
        self.joint_state_rx_time = rospy.Time.now()

        rospy.loginfo_throttle(
            1.0,
            "Joint states received: %d / %d",
            len(self.joint_positions),
            len(self.JOINT_NAMES),
        )

    def publish_square_command(self):
        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = list(self.JOINT_NAMES)
        command.position = list(self.SQUARE_POSITIONS)

        self.joint_command_pub.publish(command)

    def move_to_square(self):
        rate = rospy.Rate(self.SQUARE_PUBLISH_RATE_HZ)
        end_time = (
            rospy.Time.now()
            + rospy.Duration(self.SQUARE_PUBLISH_DURATION_SEC)
        )

        rospy.loginfo(
            "Publishing Square posture command for %.1f seconds.",
            self.SQUARE_PUBLISH_DURATION_SEC,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.publish_square_command()
            rate.sleep()
        
        rospy.loginfo("Square posture command publishing completed.")

    def start_vertex_forward_alignment(self):
        if self.demo_mode != "vertex_forward":
            return False
        
        if self.vertex_forward_target_baselink_yaw is None:
            rospy.logerr(
                "Cannot start vertex_forward alignment: "
                "target baselink yaw has not been prepared."
            )
            return False
        
        if (
            self.cog_x is None
            or self.cog_y is None
            or self.cog_z is None
        ):
            rospy.logerr(
                "Cannot start vertex_forward alignment: "
                "CoG odometry has not been received."
            )
            return False
        
        hold_x = self.cog_x
        hold_y = self.cog_y
        hold_z = self.cog_z
        target_yaw = self.vertex_forward_target_baselink_yaw

        self.publish_navigation_goal(
            hold_x,
            hold_y,
            hold_z,
            target_yaw,
        )

        rospy.loginfo(
            "Vertex-forward alignment goal published: "
            "x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
            hold_x,
            hold_y,
            hold_z,
            target_yaw,
        )
        return True
    
    def wait_for_vertex_forward_alignment(self):
        if self.vertex_forward_target_baselink_yaw is None:
            rospy.logerr(
                "Cannot wait for vertex_forward alignment: "
                "target baselink yaw has not been prepared."
            )
            return False

        rate = rospy.Rate(self.YAW_ALIGNMENT_CHECK_RATE_HZ)
        deadline = (
            rospy.Time.now()
            + rospy.Duration(self.YAW_ALIGNMENT_TIMEOUT_SEC)
        )

        rospy.loginfo(
            "Waiting for vertex-forward alignment: "
            "tolerance=%.3f rad, timeout=%.1f sec",
            self.YAW_ALIGNMENT_TOLERANCE_RAD,
            self.YAW_ALIGNMENT_TIMEOUT_SEC,
        )

        while not rospy.is_shutdown():
            if self.baselink_yaw is None:
                if rospy.Time.now() >= deadline:
                    rospy.logerr(
                        "Vertex-forward alignment timed out: "
                        "baselink odometry was not received."
                    )
                    return False
                
                rate.sleep()
                continue
            yaw_error = self.normalize_angle(
                self.vertex_forward_target_baselink_yaw
                - self.baselink_yaw
            )
            
            rospy.loginfo_throttle(
                0.5,
                "Vertex-forward alignment: "
                "current_yaw=%.3f, target_yaw=%.3f, "
                "error=%.3f rad",
                self.baselink_yaw,
                self.vertex_forward_target_baselink_yaw,
                yaw_error,
            )

            if abs(yaw_error) <= self.YAW_ALIGNMENT_TOLERANCE_RAD:
                rospy.loginfo(
                    "Vertex-forward alignment completed: "
                    "yaw_error=%.3f rad",
                    yaw_error,
                )
                return True
            
            if rospy.Time.now() >= deadline:
                rospy.logerr(
                    "Vertex-forward alignment timed out: "
                    "yaw_error=%.3f rad",
                    yaw_error
                )
                return False
            
            rate.sleep()

        return False
    
    def start_vertex_forward_motion(self):
        if self.demo_mode != "vertex_forward":
            return False
        
        if self.vertex_forward_target_baselink_yaw is None:
            rospy.logerr(
                "Cannot start vertex_forward motion: "
                "target baselink yaw has not been prepared."
                )
            return False
        
        if self.cog_z is None:
            rospy.logerr(
                "Cannot start vertex_forward motion: "
                "CoG odometry has not been received."
            )
            return False
        
        goal_x, goal_y = self.goal_world_xy
        target_z = self.cog_z
        target_yaw = self.vertex_forward_target_baselink_yaw

        self.publish_navigation_goal(
            goal_x,
            goal_y,
            target_z,
            target_yaw,
        )


        rospy.loginfo(
            "Vertex-forward navigation goal published: "
            "x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
            goal_x,
            goal_y,
            target_z,
            target_yaw,
        )
        return True

    def publish_navigation_goal(self, x, y, z, yaw):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        half_yaw = 0.5 * yaw
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(half_yaw)
        goal.pose.orientation.w = math.cos(half_yaw)

        self.navigation_goal_pub.publish(goal)

    def start_fixed_square_motion(self):
        if self.demo_mode != "fixed_square":
            rospy.loginfo(
                "Movement is not started yet for mode=%s.",
                self.demo_mode,
            )
            return
        
        if self.cog_z is None or self.cog_yaw is None:
            rospy.logerr(
                "Cannot start fixed_square motion: "
                "CoG odometry has not been received."
            )
            return
        
        goal_x, goal_y = self.goal_world_xy
        start_z = self.cog_z
        start_yaw = self.cog_yaw

        self.publish_navigation_goal(
            goal_x,
            goal_y,
            start_z,
            start_yaw
        )

        rospy.loginfo(
            "Navigation goal published: "
            "x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
            goal_x,
            goal_y,
            start_z,
            start_yaw
        )


    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
    
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def prepare_vertex_forward_alignment(self):
        if self.demo_mode != "vertex_forward":
            return False
        
        if (
            self.cog_x is None
            or self.cog_y is None
            or self.baselink_yaw is None
        ):
            rospy.logerr(
                "Cannot prepare vertex_forward alignment: "
                "required odometry has not been received."
            )
            return False
        
        goal_x, goal_y = self.goal_world_xy
        dx = goal_x - self.cog_x
        dy = goal_y - self.cog_y

        distance_to_goal = math.hypot(dx, dy)
        if distance_to_goal < 1.0e-6:
            rospy.logwarn(
                "Cannot determine travel direction: "
                "the current CoG position is already at the goal."
            )
            return False

        travel_yaw = math.atan2(dy, dx)

        current_v14_yaw = self.normalize_angle(
            self.baselink_yaw + self.V14_YAW_OFFSET
        )

        target_baselink_yaw = self.normalize_angle(
            travel_yaw - self.V14_YAW_OFFSET
        )

        yaw_change = self.normalize_angle(
            target_baselink_yaw - self.baselink_yaw
        )

        self.vertex_forward_target_baselink_yaw = (
            target_baselink_yaw
        )

        rospy.loginfo(
            "Vertex-forward yaw prepared: "
            "travel=%.3f, current_V14=%.3f, "
            "target_baselink=%.3f, yaw_change=%.3f rad",
            travel_yaw,
            current_v14_yaw,
            target_baselink_yaw,
            yaw_change,
        )
        return True
    
    
    def cog_odom_callback(self, msg):
        self.cog_x = msg.pose.pose.position.x
        self.cog_y = msg.pose.pose.position.y
        self.cog_z = msg.pose.pose.position.z

        self.cog_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        rospy.loginfo_throttle(
            1.0,
            "CoG: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
            self.cog_x,
            self.cog_y,
            self.cog_z,
            self.cog_yaw,
        )

    def baselink_odom_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )



def main():
    rospy.init_node("straight_line_demo")
    valid_modes = ("fixed_square", "vertex_forward")

    while True:
        demo_mode = input(
            "Select demo mode [fixed_square / vertex_forward]:"
        ).strip()
        
        if demo_mode in valid_modes:
            break
        print("Invalid mode. Enter fixed_square or vertex_forward.")

    while True:
        goal_text = input(
            "Enter target x y [m] in world:"
    ).strip()

        goal_parts = goal_text.split()
        if len(goal_parts) != 2:
            print("Invalid target. Enter exactly 2 numbers: x y.")
            continue

        try:
            goal_world_x = float(goal_parts[0])
            goal_world_y = float(goal_parts[1])
            break
        except ValueError:
            print("Invalid target. Enter numbers only: x y.")

    node = StraightLineDemo(
        demo_mode,
        goal_world_x,
        goal_world_y,
        )
    rospy.loginfo(
        "straight_line_demo node started: mode=%s, goal=(%.3f, %.3f)",
        node.demo_mode,
        node.goal_world_xy[0],
        node.goal_world_xy[1],
    )

    node.move_to_square()

    if node.demo_mode == "fixed_square":
        node.start_fixed_square_motion()

    elif node.demo_mode == "vertex_forward":
        prepared = node.prepare_vertex_forward_alignment()

        if prepared:
            alignment_started = (
                node.start_vertex_forward_alignment()
            )
            if alignment_started:
                alignment_completed = (
                    node.wait_for_vertex_forward_alignment()
                )
                if alignment_completed:
                    node.start_vertex_forward_motion()

    rospy.spin()

if __name__ == "__main__":
    main()