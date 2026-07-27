#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class StraightLineDemo:

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

    V14_YAW_OFFSET = 3.0 * math.pi / 4.0
    LINK4_YAW_OFFSET = math.pi / 4.0

    POSUTRE_SETTLING_TIME_SEC = 2.0


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
        )

        rospy.sleep(self.POSUTRE_SETTLING_TIME_SEC)

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
    
    def cog_odom_callback(self, msg):
        self.cog_x = msg.pose.pose.position.x
        self.cog_y = msg.pose.pose.position.y
        self.cog_z = msg.pose.pose.position.z

    def baselink_odom_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

MODE_MAP = {
    "1": "fixed_square",
    "2": "vertex_forward",
    "3": "link4_forward",
}

def read_command():
    while not rospy.is_shutdown():
        selection = input(
            "\nSelect mode "
            "[1: fixed_square / "
            "2: vertex_forward / "
            "3: link4_forward / "
            "q: quit]: "
        ).strip()

        if selection == "q":
            return None

        mode = MODE_MAP.get(selection)

        if mode is not None:
            break

        print("Invalid mode. Enter 1, 2, 3, or q.")

    while not rospy.is_shutdown():
        try:
            goal_x, goal_y = map(
                float,
                input("Enter target x y [m]: ").split(),
            )
            return mode, goal_x, goal_y
        except ValueError:
            print("Enter exactly 2 numbers: x y.")

    return None

def main():
    rospy.init_node("straight_line_demo")
    node = StraightLineDemo()

    rospy.loginfo("straight_line_demo node started.")

    while not rospy.is_shutdown():
        command = read_command()

        if command is None:
            break

        mode, goal_x, goal_y = command

        rospy.loginfo(
            "Command received: "
            "mode=%s, goal=(%.3f, %.3f)",
            mode,
            goal_x,
            goal_y,
        )

        if mode == "fixed_square":
            node.run_fixed_square(goal_x, goal_y)

        elif mode == "vertex_forward":
            node.run_vertex_forward(goal_x, goal_y)

        elif mode == "link4_forward":
            node.run_link4_forward(goal_x, goal_y)

    rospy.loginfo("Straight_line_demo terminated.")

if __name__ == "__main__":
    main()