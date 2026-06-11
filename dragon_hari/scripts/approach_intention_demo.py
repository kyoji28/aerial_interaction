#!/usr/bin/env python3

import time
import math
import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "joint1_pitch",
    "joint1_yaw",
    "joint2_pitch",
    "joint2_yaw",
    "joint3_pitch",
    "joint3_yaw",
]

# Joint order:
# position[0]: joint1_pitch
# position[1]: joint1_yaw
# position[2]: joint2_pitch
# position[3]: joint2_yaw
# position[4]: joint3_pitch
# position[5]: joint3_yaw
#
# Direction:
# pitch: positive direction bends the link downward
# yaw: positive direction bends clockwise when viewed from above

DEG_90 = math.pi / 2.0
DEG_45 = math.pi / 4.0

POSTURES = {
    # Stable square posture
    "square": [0.0, DEG_90, 0.0, DEG_90, 0.0, DEG_90],

    # Basic posture for checking pitch motion
    "pitch": [0.4, 0.2, 0.4, 0.2, 0.4, 0.2],

    # Basic posture for checking yaw motion
    "yaw": [0.0, DEG_45, 0.0, DEG_45, 0.0, DEG_45],

    # Combined pitch/yaw posture
    "combined": [0.4, 0.8, 0.4, 0.8, 0.4, 0.8],
}

# Prototype communicative motions.
# Each motion is a sequence of (posture_name, duration_sec).
#
# Note:
# These motions currently control only whole-body posture through /dragon/joints_ctrl.
# Translational motion such as forward approach or lateral avoidance is not included yet.
MOTIONS = {
    # Neutral / baseline waiting behavior
    "waiting_square_hold": [
        ("square", 5.0),
    ],

    # Approach cue: show a whole-body pre-posture before future forward motion
    "approach_combined_prepose": [
        ("square", 1.0),
        ("combined", 3.0),
        ("combined", 1.0),
    ],

    # Avoidance cue: show yaw-based turn/avoidance posture, then return to neutral
    "avoidance_yaw_turn": [
        ("square", 1.0),
        ("yaw", 3.0),
        ("yaw", 1.0),
        ("square", 2.0),
    ],
}


def publish_joint_command(pub, joint_values, duration=2.0):
    msg = JointState()
    msg.name = JOINT_NAMES
    msg.position = joint_values

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

        if (rospy.Time.now() - start_time).to_sec() > duration:
            break

        rate.sleep()


def run_motion(pub, motion_name):
    if motion_name not in MOTIONS:
        rospy.logerr("Unknown motion: %s", motion_name)
        rospy.logerr("Available motions: %s", list(MOTIONS.keys()))
        return False

    rospy.loginfo("Running motion: %s", motion_name)

    for posture_name, duration in MOTIONS[motion_name]:
        if posture_name not in POSTURES:
            rospy.logerr("Unknown posture in motion: %s", posture_name)
            return False

        rospy.loginfo(
            "Publishing posture: %s, duration: %.2f sec",
            posture_name,
            duration,
        )
        rospy.loginfo("Joint values: %s", POSTURES[posture_name])

        publish_joint_command(
            pub,
            POSTURES[posture_name],
            duration=duration,
        )

    return True


def main():
    rospy.init_node("approach_intention_demo")

    joint_control_pub = rospy.Publisher(
        "/dragon/joints_ctrl",
        JointState,
        queue_size=10,
    )

    # Wait until the publisher is ready
    time.sleep(0.6)

    # New interface:
    #   rosrun dragon_hari approach_intention_demo.py _motion:=waiting_square_hold
    #
    # Backward-compatible interface:
    #   rosrun dragon_hari approach_intention_demo.py _posture:=square
    motion_name = rospy.get_param("~motion", None)
    posture_name = rospy.get_param("~posture", None)

    if motion_name is not None:
        success = run_motion(joint_control_pub, motion_name)
        if success:
            rospy.loginfo("Finished publishing motion command.")
        return

    if posture_name is None:
        posture_name = "square"

    if posture_name not in POSTURES:
        rospy.logerr("Unknown posture: %s", posture_name)
        rospy.logerr("Available postures: %s", list(POSTURES.keys()))
        return

    rospy.loginfo("Publishing single posture: %s", posture_name)
    rospy.loginfo("Joint values: %s", POSTURES[posture_name])

    publish_joint_command(
        joint_control_pub,
        POSTURES[posture_name],
        duration=2.0,
    )

    rospy.loginfo("Finished publishing posture command.")


if __name__ == "__main__":
    main()
