#!/usr/bin/env python3

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

def main():
    rospy.init_node("joint_angle_input")

    joint_pub = rospy.Publisher(
        "/dragon/joints_ctrl",
        JointState,
        queue_size=1,
    )

    print("Waiting for /dragon/joints_ctrl subscriber...")

    while (
        joint_pub.get_num_connections() == 0
        and not rospy.is_shutdown()
    ):
        rospy.sleep(0.1)

    print("Connected.")
    print()
    print("Enter six joint angles in degrees:")
    print(
        "joint1_pitch joint1_yaw"
        "joint2_pitch joint2_yaw"
        "joint3_pitch joint3_yaw"
        )
    print("Example: 35 90 0 90 0 75")
    print("Enter q to quit.")

    while not rospy.is_shutdown():
        user_input = input("\nJoint angles [deg]: ").strip()

        if user_input.lower() in ("q", "quit"):
                break

        values = user_input.split()

        if len(values) != 6:
            print("Enter exactly 6 angle values.")
            continue

        try:
            angles_deg = [float(value) for value in values]
        except ValueError:
            print("All inputs must be numbers.")
            continue

        if any(abs(angle) > 90.0 for angle in angles_deg):
            print("Each joint angle must be between -90 and 90 degrees.")
            continue

        angles_rad = [
            math.radians(angle)
            for angle in angles_deg
            ]

        command = JointState()
        command.header.stamp = rospy.Time.now()
        command.name = JOINT_NAMES
        command.position = angles_rad

        joint_pub.publish(command)

        print("Published:")
        for name, angle_deg, angle_rad in zip(
            JOINT_NAMES,
            angles_deg,
            angles_rad,
            ):
            print(
                " {}: {:.1f} deg ({:.4f} rad)".format(
                    name,
                    angle_deg,
                    angle_rad,
                    )
                )

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass