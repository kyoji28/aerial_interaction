#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class StraightLineBodyWave:
    RATE_HZ = 20.0
    YAW_TOLERANCE_RAD = math.radians(5.0)
    POSITION_TOLERANCE_M = 0.10
    POSTURE_SETTLING_TIME_SEC = 2.0

    JOINT_NAMES = (
        "joint1_pitch", "joint1_yaw",
        "joint2_pitch", "joint2_yaw",
        "joint3_pitch", "joint3_yaw",
    )

    BASE_POSITIONS = (
        0.0, math.radians(80.0),
        0.0, math.radians(80.0),
        0.0, math.radians(-35.0),
    )

    YAW_OFFSET_RAD = math.radians(35.0)
    WAVE_AMPLITUDE_RAD = math.radians(10.0)
    WAVE_CYCLES = 5.0
    PHASE_DELAY_RAD = math.pi / 3.0

    def __init__(self):
        self.cog_x = self.cog_y = self.cog_z = None
        self.baselink_yaw = None

        self.joint_pub = rospy.Publisher(
            "/dragon/joints_ctrl",
            JointState,
            queue_size=10,
        )
        self.goal_pub = rospy.Publisher(
            "/dragon/target_pose",
            PoseStamped,
            queue_size=1,
        )

        self.cog_sub = rospy.Subscriber(
            "/dragon/uav/cog/odom",
            Odometry,
            self.cog_callback,
            queue_size=1,
        )
        self.baselink_sub = rospy.Subscriber(
            "/dragon/uav/baselink/odom",
            Odometry,
            self.baselink_callback,
            queue_size=1,
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def clamp(value):
        return max(0.0, min(value, 1.0))

    def cog_callback(self, msg):
        position = msg.pose.pose.position
        self.cog_x, self.cog_y, self.cog_z = (
            position.x,
            position.y,
            position.z,
        )

    def baselink_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def publish_joints(self, positions):
        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = list(self.JOINT_NAMES)
        command.position = list(positions)
        self.joint_pub.publish(command)

    def publish_goal(self, x, y, z, yaw):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)

    def wait_until_ready(self):
        rospy.loginfo(
            "Waiting for odometry and "
            "/dragon/joints_ctrl connection."
        )
        rate = rospy.Rate(self.RATE_HZ)

        while not rospy.is_shutdown():
            state_ready = None not in (
                self.cog_x,
                self.cog_y,
                self.cog_z,
                self.baselink_yaw,
            )
            command_ready = self.joint_pub.get_num_connections() > 0

            if state_ready and command_ready:
                rospy.loginfo(
                    "Body-wave demo is ready: "
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

    def move_to_base_posture(self):
        rospy.loginfo("Moving to body-wave base posture.")
        self.publish_joints(self.BASE_POSITIONS)

        rospy.loginfo(
            "Waiting %.1f seconds for posture settling.",
            self.POSTURE_SETTLING_TIME_SEC,
        )
        rospy.sleep(self.POSTURE_SETTLING_TIME_SEC)

        rospy.loginfo(
            "Body-wave base posture settling completed."
        )

    def body_wave_positions(self, progress):
        progress = self.clamp(progress)
        phase = 2.0 * math.pi * self.WAVE_CYCLES * progress
        amplitude = (
            self.WAVE_AMPLITUDE_RAD
            * math.sin(math.pi * progress) ** 2
        )

        base_yaws = self.BASE_POSITIONS[1::2]
        delays = (
            2.0 * self.PHASE_DELAY_RAD,
            self.PHASE_DELAY_RAD,
            0.0,
        )

        yaws = tuple(
            base + amplitude * math.sin(phase - delay)
            for base, delay in zip(base_yaws, delays)
        )

        return (
            0.0, yaws[0],
            0.0, yaws[1],
            0.0, yaws[2],
        )

    def path_progress(
        self,
        start_x,
        start_y,
        goal_x,
        goal_y,
    ):
        path_x = goal_x - start_x
        path_y = goal_y - start_y
        length_squared = path_x**2 + path_y**2

        if length_squared <= 1.0e-12:
            return 1.0

        progress = (
            (self.cog_x - start_x) * path_x
            + (self.cog_y - start_y) * path_y
        ) / length_squared

        return self.clamp(progress)

    def target_yaw(self, goal_x, goal_y):
        travel_yaw = math.atan2(
            goal_y - self.cog_y,
            goal_x - self.cog_x,
        )
        target_yaw = self.normalize_angle(
            travel_yaw - self.YAW_OFFSET_RAD
        )

        rospy.loginfo(
            "Body-wave yaw: travel=%.3f rad, "
            "offset=%.3f rad, "
            "target_baselink=%.3f rad",
            travel_yaw,
            self.YAW_OFFSET_RAD,
            target_yaw,
        )

        return target_yaw

    def align_yaw(self, target_yaw):
        self.publish_goal(
            self.cog_x,
            self.cog_y,
            self.cog_z,
            target_yaw,
        )
        rate = rospy.Rate(self.RATE_HZ)

        while not rospy.is_shutdown():
            error = self.normalize_angle(
                target_yaw - self.baselink_yaw
            )

            rospy.loginfo_throttle(
                0.5,
                "Yaw error: %.3f rad",
                error,
            )

            if abs(error) <= self.YAW_TOLERANCE_RAD:
                rospy.loginfo("Yaw alignment completed.")
                return True

            rate.sleep()

        return False

    def move_with_body_wave(self, goal_x, goal_y):
        start_x, start_y = self.cog_x, self.cog_y
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
            return True

        self.publish_goal(
            goal_x,
            goal_y,
            self.cog_z,
            self.baselink_yaw,
        )
        rate = rospy.Rate(self.RATE_HZ)

        try:
            while not rospy.is_shutdown():
                distance = math.hypot(
                    goal_x - self.cog_x,
                    goal_y - self.cog_y,
                )
                progress = self.path_progress(
                    start_x,
                    start_y,
                    goal_x,
                    goal_y,
                )

                self.publish_joints(
                    self.body_wave_positions(progress)
                )

                rospy.loginfo_throttle(
                    0.5,
                    "Distance to goal: %.3f m, "
                    "progress=%.3f",
                    distance,
                    progress,
                )

                if distance <= self.POSITION_TOLERANCE_M:
                    rospy.loginfo(
                        "Goal reached: distance=%.3f m, "
                        "progress=%.3f",
                        distance,
                        progress,
                    )
                    return True

                rate.sleep()

            return False

        finally:
            if not rospy.is_shutdown():
                self.publish_joints(self.BASE_POSITIONS)

    def run(self, goal_x, goal_y):
        rospy.loginfo("Starting body_wave demo.")
        self.move_to_base_posture()

        if not self.align_yaw(
            self.target_yaw(goal_x, goal_y)
        ):
            return False

        completed = self.move_with_body_wave(
            goal_x,
            goal_y,
        )

        if completed:
            rospy.loginfo("body_wave demo completed.")

        return completed


def read_goal():
    while not rospy.is_shutdown():
        try:
            values = input(
                "Enter target x y [m]: "
            ).split()

            if len(values) != 2:
                raise ValueError

            return tuple(map(float, values))

        except ValueError:
            print("Enter exactly 2 numbers: x y.")

        except (EOFError, KeyboardInterrupt):
            return None

    return None


def main():
    rospy.init_node("straight_line_body_wave")

    node = StraightLineBodyWave()
    rospy.loginfo(
        "straight_line_body_wave node started."
    )

    if not node.wait_until_ready():
        return

    goal = read_goal()

    if goal is None:
        return

    rospy.loginfo(
        "Goal received: (%.3f, %.3f)",
        *goal,
    )

    if not node.run(*goal):
        rospy.loginfo(
            "Body-wave demo did not complete."
        )

    rospy.loginfo(
        "straight_line_body_wave terminated."
    )


if __name__ == "__main__":
    main()
