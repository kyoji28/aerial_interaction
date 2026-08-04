#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class StraightLineFixedSquare:

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
        if self.joint_command_pub.get_num_connections() == 0:
            rospy.loginfo(
                "Waiting for /dragon/joints_ctrl connection."
            )

        while (
            not rospy.is_shutdown()
            and self.joint_command_pub.get_num_connections() == 0
        ):
            rospy.sleep(0.05)

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

    def cog_odom_callback(self, msg):
        self.cog_x = msg.pose.pose.position.x
        self.cog_y = msg.pose.pose.position.y
        self.cog_z = msg.pose.pose.position.z

    def baselink_odom_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )


def read_goal():
    while not rospy.is_shutdown():
        try:
            goal_x, goal_y = map(
                float,
                input("Enter target x y [m]: ").split(),
            )
            return goal_x, goal_y

        except ValueError:
            print("Enter exactly 2 numbers: x y.")

    return None


def main():
    rospy.init_node("straight_line_fixed_square")

    node = StraightLineFixedSquare()

    rospy.loginfo(
        "straight_line_fixed_square node started."
    )

    goal = read_goal()

    if goal is None:
        return

    goal_x, goal_y = goal

    rospy.loginfo(
        "Goal received: (%.3f, %.3f)",
        goal_x,
        goal_y,
    )

    completed = node.run_fixed_square(
        goal_x,
        goal_y,
    )

    if not completed:
        rospy.loginfo(
            "Fixed-square demo did not complete."
        )

    rospy.loginfo(
        "straight_line_fixed_square terminated."
    )


if __name__ == "__main__":
    main()
