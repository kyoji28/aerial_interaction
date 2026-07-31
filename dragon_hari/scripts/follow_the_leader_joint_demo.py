#!/usr/bin/env python3

import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState

from follow_the_leader_trajectory import (
    JOINT_NAMES,
    build_timed_trajectory,
)


JOINT_COMMAND_TOPIC = "/dragon/joints_ctrl"
JOINT_STATE_TOPIC = "/dragon/joint_states"


class FollowTheLeaderJointDemo:
    def __init__(self):
        self.joint_positions = {}
        self.joint_state_lock = threading.Lock()

        self.joint_command_publisher = rospy.Publisher(
            JOINT_COMMAND_TOPIC,
            JointState,
            queue_size=1,
        )

        self.joint_state_subscriber = rospy.Subscriber(
            JOINT_STATE_TOPIC,
            JointState,
            self.joint_state_callback,
            queue_size=1,
        )

    def joint_state_callback(self, message):
        with self.joint_state_lock:
            for name, position in zip(
                message.name,
                message.position,
            ):
                if name in JOINT_NAMES:
                    self.joint_positions[name] = position

    def get_current_joint_positions(self):
        with self.joint_state_lock:
            missing_joint_names = [
                name
                for name in JOINT_NAMES
                if name not in self.joint_positions
            ]

            if missing_joint_names:
                return None

            return [
                self.joint_positions[name]
                for name in JOINT_NAMES
            ]

    def wait_for_joint_states(self, timeout):
        start_time = time.monotonic()

        while not rospy.is_shutdown():
            joint_positions = (
                self.get_current_joint_positions()
            )

            if joint_positions is not None:
                rospy.loginfo(
                    "Joint states received: %d / %d",
                    len(joint_positions),
                    len(JOINT_NAMES),
                )

                return joint_positions

            if time.monotonic() - start_time > timeout:
                raise RuntimeError(
                    "Timed out while waiting for "
                    "all required joint states."
                )

            rospy.sleep(0.05)

        raise rospy.ROSInterruptException(
            "ROS shutdown while waiting "
            "for joint states."
        )

    def publish_joint_positions(
        self,
        joint_positions,
    ):
        message = JointState()

        message.header.stamp = rospy.Time.now()
        message.name = list(JOINT_NAMES)
        message.position = list(joint_positions)

        self.joint_command_publisher.publish(
            message
        )

    def interpolate_to_first_command(
        self,
        start_positions,
        target_positions,
        duration,
        control_rate,
    ):
        if duration <= 0.0:
            self.publish_joint_positions(
                target_positions
            )
            return

        number_of_steps = max(
            1,
            int(math.ceil(
                duration * control_rate
            )),
        )

        rate = rospy.Rate(control_rate)

        rospy.loginfo(
            "Interpolating to the first command "
            "over %.2f seconds.",
            duration,
        )

        for step_index in range(
            number_of_steps + 1
        ):
            if rospy.is_shutdown():
                return

            normalized_time = (
                step_index
                / number_of_steps
            )

            blend = (
                10.0 * normalized_time**3
                - 15.0 * normalized_time**4
                + 6.0 * normalized_time**5
            )

            command_positions = [
                start
                + blend * (target - start)
                for start, target in zip(
                    start_positions,
                    target_positions,
                )
            ]

            self.publish_joint_positions(
                command_positions
            )

            rate.sleep()

    def hold_command(
        self,
        joint_positions,
        duration,
        control_rate,
    ):
        number_of_steps = max(
            1,
            int(math.ceil(
                duration * control_rate
            )),
        )

        rate = rospy.Rate(control_rate)

        for _ in range(number_of_steps):
            if rospy.is_shutdown():
                return

            self.publish_joint_positions(
                joint_positions
            )

            rate.sleep()

    def play_trajectory(
        self,
        trajectory,
        control_rate,
    ):
        rate = rospy.Rate(control_rate)

        rospy.loginfo(
            "Playing joint-only trajectory: "
            "%d samples at %.1f Hz.",
            len(trajectory),
            control_rate,
        )

        for sample_index, command in enumerate(
            trajectory
        ):
            if rospy.is_shutdown():
                return

            self.publish_joint_positions(
                command["joint_positions"]
            )

            if sample_index % int(control_rate) == 0:
                rospy.loginfo(
                    "Playback progress: %.2f / %.2f s",
                    command["time"],
                    trajectory[-1]["time"],
                )

            rate.sleep()


def main():
    rospy.init_node(
        "follow_the_leader_joint_demo"
    )

    end_tip_speed = rospy.get_param(
        "~end_tip_speed",
        0.10,
    )

    control_rate = rospy.get_param(
        "~control_rate",
        20.0,
    )

    transition_duration = rospy.get_param(
        "~transition_duration",
        4.0,
    )

    initial_hold_duration = rospy.get_param(
        "~initial_hold_duration",
        1.0,
    )

    final_hold_duration = rospy.get_param(
        "~final_hold_duration",
        2.0,
    )

    joint_state_timeout = rospy.get_param(
        "~joint_state_timeout",
        10.0,
    )

    demo = FollowTheLeaderJointDemo()

    trajectory, _, _ = build_timed_trajectory(
        end_tip_speed=end_tip_speed,
        control_rate=control_rate,
    )

    rospy.loginfo(
        "Joint-only follow-the-leader demo started."
    )

    rospy.loginfo(
        "This node publishes only to %s.",
        JOINT_COMMAND_TOPIC,
    )

    current_joint_positions = (
        demo.wait_for_joint_states(
            timeout=joint_state_timeout
        )
    )

    first_joint_positions = trajectory[0][
        "joint_positions"
    ]

    demo.interpolate_to_first_command(
        start_positions=current_joint_positions,
        target_positions=first_joint_positions,
        duration=transition_duration,
        control_rate=control_rate,
    )

    rospy.loginfo(
        "Holding the first command for %.2f seconds.",
        initial_hold_duration,
    )

    demo.hold_command(
        joint_positions=first_joint_positions,
        duration=initial_hold_duration,
        control_rate=control_rate,
    )

    demo.play_trajectory(
        trajectory=trajectory,
        control_rate=control_rate,
    )

    final_joint_positions = trajectory[-1][
        "joint_positions"
    ]

    rospy.loginfo(
        "Holding the final command for %.2f seconds.",
        final_hold_duration,
    )

    demo.hold_command(
        joint_positions=final_joint_positions,
        duration=final_hold_duration,
        control_rate=control_rate,
    )

    rospy.loginfo(
        "Joint-only trajectory playback completed."
    )


if __name__ == "__main__":
    try:
        main()
    except (
        rospy.ROSInterruptException,
        RuntimeError,
        ValueError,
    ) as error:
        rospy.logerr(str(error))