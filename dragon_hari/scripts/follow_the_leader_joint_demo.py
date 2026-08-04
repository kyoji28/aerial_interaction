#!/usr/bin/env python3

import rospy

from dragon_hari.motions.follow_the_leader.joint_player import (
    JOINT_COMMAND_TOPIC,
    FollowTheLeaderJointDemo,
)
from dragon_hari.motions.follow_the_leader.trajectory import (
    build_timed_trajectory,
)


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
