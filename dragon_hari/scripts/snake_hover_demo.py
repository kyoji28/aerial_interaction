#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class SnakeHoverDemo:

    COMMAND_RATE_HZ = 20.0
    STATE_CHECK_RATE_HZ = 20.0
    POSTURE_SETTLING_TIME_SEC = 2.0

    WAVE_AMPLITUDE_RAD = math.radians(10.0)
    WAVE_PERIOD_SEC = 8.0
    PHASE_DELAY_RAD = math.pi / 3.0

    RAMP_DURATION_SEC = 2.0
    DEMO_DURATION_SEC = 16.0

    JOINT_NAMES = (
        "joint1_pitch",
        "joint1_yaw",
        "joint2_pitch",
        "joint2_yaw",
        "joint3_pitch",
        "joint3_yaw",
    )

    BASE_POSITIONS = (
        0.0,
        math.radians(80.0),
        0.0,
        math.radians(80.0),
        0.0,
        math.radians(-35.0),
    )

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

    @staticmethod
    def quaternion_to_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def cog_odom_callback(self, msg):
        self.cog_x = msg.pose.pose.position.x
        self.cog_y = msg.pose.pose.position.y
        self.cog_z = msg.pose.pose.position.z

    def baselink_odom_callback(self, msg):
        self.baselink_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def wait_for_initial_state(self):
        rospy.loginfo("Waiting for CoG and baselink odometry.")
        rate = rospy.Rate(self.STATE_CHECK_RATE_HZ)

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
                    "CoG =(%.3f, %.3f, %.3f), "
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

        rate = rospy.Rate(self.STATE_CHECK_RATE_HZ)

        while not rospy.is_shutdown():
            if self.joint_command_pub.get_num_connections() > 0:
                rospy.loginfo(
                    "/dragon/joints_ctrl subscriber connected."
                )
                return True

            rate.sleep()

        return False


    def publish_joint_command(self, positions):
        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = list(self.JOINT_NAMES)
        command.position = list(positions)

        self.joint_command_pub.publish(command)

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

    def move_to_base_posture(self):
        rospy.loginfo("Moving to body-wave base posture.")

        hold_x = self.cog_x
        hold_y = self.cog_y
        hold_z = self.cog_z
        hold_yaw = self.baselink_yaw

        self.publish_navigation_goal(
            hold_x,
            hold_y,
            hold_z,
            hold_yaw,
        )

        self.publish_joint_command(
            self.BASE_POSITIONS
        )

        rospy.sleep(self.POSTURE_SETTLING_TIME_SEC)
        rospy.loginfo("Body-wave base posture completed.")

    def calculate_ramp_scale(
            self,
            elapsed_time,
    ):
        if elapsed_time <= 0.0:
            return 0.0

        if elapsed_time < self.RAMP_DURATION_SEC:
            return 0.5 * (
                1.0
                - math.cos(
                    math.pi
                    * elapsed_time
                    / self.RAMP_DURATION_SEC
                )
            )

        remaining_time = (
            self.DEMO_DURATION_SEC
            - elapsed_time
        )

        if remaining_time <= 0.0:
            return 0.0

        if remaining_time < self.RAMP_DURATION_SEC:
            return 0.5 * (
                1.0
                - math.cos(
                    math.pi
                    * remaining_time
                    / self.RAMP_DURATION_SEC
                )
            )
        return 1.0

    def calculate_body_wave_positions(
            self,
            elapsed_time,
    ):
        phase = (
            2.0
            * math.pi
            * elapsed_time
            / self.WAVE_PERIOD_SEC
        )

        ramp_scale = self.calculate_ramp_scale(
            elapsed_time
        )

        wave_amplitude = (
            ramp_scale
            * self.WAVE_AMPLITUDE_RAD
        )

        joint3_yaw = (
            self.BASE_POSITIONS[5]
            + wave_amplitude
            * math.sin(phase)
        )

        joint2_yaw = (
            self.BASE_POSITIONS[3]
            + wave_amplitude
            * math.sin(
                phase
                - self.PHASE_DELAY_RAD
            )
        )

        joint1_yaw = (
            self.BASE_POSITIONS[1]
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

    def run_body_wave(self):
        rospy.loginfo(
            "Starting body wave: "
            "amplitude=%.1f deg, period=%.1f s, "
            "duration=%.1f s.",
            math.degrees(self.WAVE_AMPLITUDE_RAD),
            self.WAVE_PERIOD_SEC,
            self.DEMO_DURATION_SEC,
        )

        rate = rospy.Rate(self.COMMAND_RATE_HZ)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed_time = (
                rospy.Time.now() - start_time
            ).to_sec()

            if elapsed_time >= self.DEMO_DURATION_SEC:
                break

            positions = (
                self.calculate_body_wave_positions(
                    elapsed_time
                )
            )

            self.publish_joint_command(positions)

            rate.sleep()

        if rospy.is_shutdown():
            return False

        self.publish_joint_command(
            self.BASE_POSITIONS
        )

        rospy.loginfo(
            "Body wave completed. "
            "Returned to base posture."
        )

        return True


def main():
    rospy.init_node("snake_hover_demo")
    node = SnakeHoverDemo()

    rospy.loginfo("snake_hover_demo node started.")

    if not node.wait_for_initial_state():
        return

    if not node.wait_for_command_connection():
        return

    node.move_to_base_posture()

    if not node.run_body_wave():
        return

    rospy.loginfo(
        "Body-wave hover test completed. "
        "Press Ctrl+C to terminate."
    )

    rospy.spin()

if __name__ == "__main__":
    main()