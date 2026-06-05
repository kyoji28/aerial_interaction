#!/usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node("approach_intention_demo")

    joint_control_pub = rospy.Publisher(
        "/dragon/joints_ctrl",
        JointState,
        queue_size=10
        )
    # Wait until the publisher is ready
    time.sleep(0.6)

    desire_joint = JointState()

    # Minimal posture command for checking whether DRAGON changes posture.
    # Joint order follows robots/dragon/scripts/transformation_demo.py.
    desire_joint.position = [-0.4, 1.2, -0.4, 1.2, -0.4, 1.2]

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        desire_joint.header.stamp = rospy.Time.now()
        joint_control_pub.publish(desire_joint)

        if (rospy.Time.now() - start_time).to_sec() > 2.0:
            break

        rate.sleep()

    rospy.loginfo("Published approach intention posture command.")

if __name__ == "__main__":
    main()
