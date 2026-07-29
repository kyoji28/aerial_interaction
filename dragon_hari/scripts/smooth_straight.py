#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class SmoothStraight:

    STATE_CHECK_RATE_HZ = 20.0
    YAW_ALIGNMENT_TOLERANCE_RAD = math.radians(5.0)
    YAW_ALIGNMENT_CHECK_RATE_HZ = 20.0

    POSITION_TOLERANCE_M = 0.10
    POSITION_CHECK_RATE_HZ = 20.0

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

    LINK4_FORWARD_POSITIONS = (
        0.0,
        math.pi / 2.0,
        0.0,
        math.pi / 2.0,
        0.0,
        -math.pi / 4.0,
    )

    SMOOTH_STRAIGHT_BASE_POSITIONS = (
        0.0,
        math.radians(80.0),
        0.0,
        math.radians(80.0),
        0.0,
        math.radians(-35.0),
    )

    WAVE_AMPLITUDE_RAD = math.radians(10.0)
    WAVE_CYCLES = 5.0
    PHASE_DELAY_RAD = math.pi / 3.0

    V14_YAW_OFFSET = 3.0 * math.pi / 4.0
    LINK4_YAW_OFFSET = math.pi / 4.0

    POSTURE_SETTLING_TIME_SEC = 2.0


    def __init__(self):
        self.cog_x = None
        self.cog_y = None
        self.cog_z = None
        self.baselink_yaw = None


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

    def publish_joint_command(self, positions):
        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = list(self.JOINT_NAMES)
        command.position = list(positions)

        self.joint_command_pub.publish(command)


    def move_to_posture(self, positions, posture_name):
        rospy.loginfo(
            "Moving to %s posture.",
            posture_name,
        )

        self.publish_joint_command(positions)

        rospy.loginfo(
            "Waiting %.1f seconds for posture settling.",
            self.POSTURE_SETTLING_TIME_SEC,
        )

        rospy.sleep(self.POSTURE_SETTLING_TIME_SEC)

        rospy.loginfo(
            "%s posture settling completed.",
            posture_name,
        )


    def move_to_square(self):
        self.move_to_posture(
            self.SQUARE_POSITIONS,
            "Square",
        )

    def move_to_link4_forward_posture(self):
        self.move_to_posture(
            self.LINK4_FORWARD_POSITIONS,
            "Link4-forward",
        )

    def move_to_smooth_straight_base_posture(self):
        self.move_to_posture(
            self.SMOOTH_STRAIGHT_BASE_POSITIONS,
            "Smooth-straight base",
        )

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


    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
    
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(
            minimum,
            min(value, maximum),
        )

    def calculate_body_wave_positions(
            self,
            progress,
            ):
        progress = self.clamp(
            progress,
            0.0,
            1.0,
        )

        phase = (
            2.0
            * math.pi
            * self.WAVE_CYCLES
            * progress
        )

        envelope = math.sin(
            math.pi * progress
        ) ** 2

        wave_amplitude = (
            self.WAVE_AMPLITUDE_RAD
            * envelope
        )

        joint3_yaw = (
            self.SMOOTH_STRAIGHT_BASE_POSITIONS[5]
            + wave_amplitude
            * math.sin(phase)
        )

        joint2_yaw = (
            self.SMOOTH_STRAIGHT_BASE_POSITIONS[3]
            + wave_amplitude
            * math.sin(
                phase
                - self.PHASE_DELAY_RAD
            )
        )

        joint1_yaw = (
            self.SMOOTH_STRAIGHT_BASE_POSITIONS[1]
            + wave_amplitude
            * math.sin(
                phase
                - 2.0
                * self.PHASE_DELAY_RAD
            )
        )

        return (
            0.0,
            joint1_yaw,
            0.0,
            joint2_yaw,
            0.0,
            joint3_yaw,
        )

    def calculate_path_progress(
            self,
            start_x,
            start_y,
            goal_x,
            goal_y,
    ):
        path_x = goal_x - start_x
        path_y = goal_y - start_y

        path_length_squared = (
            path_x * path_x
            + path_y * path_y
        )

        if path_length_squared <= 1.0e-12:
            return 1.0

        traveled_x = self.cog_x - start_x
        traveled_y = self.cog_y - start_y

        progress = (
            traveled_x * path_x
            + traveled_y * path_y
        ) / path_length_squared

        return self.clamp(
            progress,
            0.0,
            1.0,
        )
    
    def calculate_vertex_forward_yaw(self, goal_x, goal_y):
        travel_yaw = math.atan2(
            goal_y - self.cog_y,
            goal_x - self.cog_x,
        )

        return self.normalize_angle(
            travel_yaw - self.V14_YAW_OFFSET
        )

    def calculate_link4_forward_yaw(self, goal_x, goal_y):
        travel_yaw = math.atan2(
            goal_y - self.cog_y,
            goal_x - self.cog_x,
        )

        target_baselink_yaw = self.normalize_angle(
            travel_yaw - self.LINK4_YAW_OFFSET
        )

        rospy.loginfo(
            "Link4-forward yaw: "
            "travel=%.3f rad, "
            "offset=%.3f rad, "
            "target_baselink=%.3f rad",
            travel_yaw,
            self.LINK4_YAW_OFFSET,
            target_baselink_yaw,
        )

        return target_baselink_yaw


    def align_yaw(self, target_yaw):
        self.publish_navigation_goal(
            self.cog_x,
            self.cog_y,
            self.cog_z,
            target_yaw,
        )

        rate = rospy.Rate(self.YAW_ALIGNMENT_CHECK_RATE_HZ)

        while not rospy.is_shutdown():
            yaw_error = self.normalize_angle(
                target_yaw - self.baselink_yaw
            )

            rospy.loginfo_throttle(
                0.5,
                "Yaw error: %.3f rad",
                yaw_error,
            )

            if abs(yaw_error) <= self.YAW_ALIGNMENT_TOLERANCE_RAD:
                rospy.loginfo("Yaw alignment completed.")
                return True

            rate.sleep()

        return False
    

    def move_to_goal(self, goal_x, goal_y):
        target_yaw = self.baselink_yaw

        self.publish_navigation_goal(
            goal_x,
            goal_y,
            self.cog_z,
            target_yaw,
        )

        rate = rospy.Rate(self.POSITION_CHECK_RATE_HZ)

        while not rospy.is_shutdown():
            distance = math.hypot(
                goal_x - self.cog_x,
                goal_y - self.cog_y,
            )

            rospy.loginfo_throttle(
                0.5,
                "Distance to goal: %.3f m",
                distance,
            )

            if distance <= self.POSITION_TOLERANCE_M:
                rospy.loginfo(
                    "Goal reached: distance=%.3f m",
                    distance,
                )
                return True
            
            rate.sleep()

        return False

    def move_to_goal_with_body_wave(
            self,
            goal_x,
            goal_y,
    ):
        start_x = self.cog_x
        start_y = self.cog_y
        target_z = self.cog_z
        target_yaw = self.baselink_yaw

        initial_distance = math.hypot(
            goal_x - start_x,
            goal_y - start_y,
        )

        if initial_distance <= self.POSITION_TOLERANCE_M:
            rospy.loginfo(
                "Robot is already near the goal: "
                "distance=%.3f m",
                initial_distance,
            )
            self.publish_joint_command(
                self.SMOOTH_STRAIGHT_BASE_POSITIONS
            )
            return True

        self.publish_navigation_goal(
            goal_x,
            goal_y,
            target_z,
            target_yaw,
        )

        rate = rospy.Rate(
            self.POSITION_CHECK_RATE_HZ
        )

        while not rospy.is_shutdown():
            distance = math.hypot(
                goal_x - self.cog_x,
                goal_y - self.cog_y,
            )

            progress = self.calculate_path_progress(
                start_x,
                start_y,
                goal_x,
                goal_y,
            )

            positions = (
                self.calculate_body_wave_positions(
                    progress
                )
            )

            self.publish_joint_command(
                positions
            )

            rospy.loginfo_throttle(
                0.5,
                "Distance to goal: %.3f m, "
                "progress=%.3f",
                distance,
                progress,
            )

            if distance <= self.POSITION_TOLERANCE_M:
                self.publish_joint_command(
                    self.SMOOTH_STRAIGHT_BASE_POSITIONS
                )

                rospy.loginfo(
                    "Goal reached: "
                    "distance=%.3f m, "
                    "progress=%.3f",
                    distance,
                    progress,
                )

                return True

            rate.sleep()

        self.publish_joint_command(
            self.SMOOTH_STRAIGHT_BASE_POSITIONS
        )

        return False


    def run_fixed_square(self, goal_x, goal_y):
        rospy.loginfo("Starting fixed_square demo.")

        self.move_to_square()
        completed = self.move_to_goal(goal_x, goal_y)

        if completed:
            rospy.loginfo("fixed_square demo completed.")

        return completed

    def run_vertex_forward(self, goal_x, goal_y):
        rospy.loginfo("Starting vertex_forward demo.")

        self.move_to_square()

        target_yaw = self.calculate_vertex_forward_yaw(
            goal_x,
            goal_y,
        )

        if not self.align_yaw(target_yaw):
            return False

        completed = self.move_to_goal(goal_x, goal_y)

        if completed:
            rospy.loginfo("vertex_forward demo completed.")

        return completed

    def run_link4_forward(self, goal_x, goal_y):
        rospy.loginfo("Starting link4_forward demo.")

        self.move_to_link4_forward_posture()

        target_yaw = self.calculate_link4_forward_yaw(
            goal_x,
            goal_y,
        )

        if not self.align_yaw(target_yaw):
            return False

        completed = self.move_to_goal(goal_x, goal_y)

        if completed:
            rospy.loginfo("link4_forward demo completed.")

        return completed

    def run_smooth_straight(self, goal_x, goal_y):
        rospy.loginfo("Starting smooth-straigh baseline test.")

        self.move_to_smooth_straight_base_posture()

        target_yaw = self.calculate_link4_forward_yaw(
            goal_x,
            goal_y,
        )

        if not self.align_yaw(target_yaw):
            return False

        completed = self.move_to_goal_with_body_wave(
            goal_x,
            goal_y,
        )

        if completed:
            rospy.loginfo(
                "Smooth-straight baseline test completed."
            )

        return completed
    
    def cog_odom_callback(self, msg):
        self.cog_x = msg.pose.pose.position.x
        self.cog_y = msg.pose.pose.position.y
        self.cog_z = msg.pose.pose.position.z

    def baselink_odom_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def wait_for_initial_state(self):
        rospy.loginfo(
            "Waiting for CoG and baselink odometry."
        )

        rate = rospy.Rate(
            self.STATE_CHECK_RATE_HZ
        )

        while not rospy.is_shutdown():
            state_ready = (
                self.cog_x is not None
                and self.cog_y is not None
                and self.cog_z is not None
                and self.baselink_yaw is not None
            )

            if state_ready:
                rospy.loginfo(
                    "Initial state received: "
                    "CoG=(%.3f, %.3f, %.3f), "
                    "baselink_yaw=%.3f rad",
                    self.cog_x,
                    self.cog_y,
                    self.cog_z,
                    self.baselink_yaw,
                )
                return True

            rate.sleep()

        return False

    def wait_for_command_connection(self):
        rospy.loginfo(
            "Waiting for /dragon/joints_ctrl subscriber."
        )

        rate = rospy.Rate(
            self.STATE_CHECK_RATE_HZ
        )

        while not rospy.is_shutdown():
            if (
                self.joint_command_pub
                .get_num_connections()
                > 0
            ):
                rospy.loginfo(
                    "/dragon/joints_ctrl "
                    "subscriber connected."
                )
                return True

            rate.sleep()

        return False

def read_goal():
    while not rospy.is_shutdown():
        try:
            goal_x, goal_y = map(
                float,
                input(
                    "Enter target x y [m]: "
                ).split(),
            )

            return goal_x, goal_y
        
        except ValueError:
            print(
                "Enter exactly 2 numbers: x y."
            )

    return None

def main():
    rospy.init_node("smooth_straight")
    node = SmoothStraight()
    rospy.loginfo("smooth_straight node started.")
    if not node.wait_for_initial_state():
        return

    if not node.wait_for_command_connection():
        return

    goal = read_goal()

    if goal is None:
        return

    goal_x, goal_y = goal

    rospy.loginfo(
        "Goal received: (%.3f, %.3f)",
        goal_x,
        goal_y,
    )

    completed = node.run_smooth_straight(
        goal_x,
        goal_y,
    )

    if not completed:
        rospy.loginfo(
            "Smooth-straight baseline test "
            "did not complete."
        )

    rospy.loginfo("smooth_straight terminated.")

if __name__ == "__main__":
    main()