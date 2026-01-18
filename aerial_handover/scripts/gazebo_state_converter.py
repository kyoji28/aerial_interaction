#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from gazebo_msgs.msg import LinkState, LinkStates


class PoseToGazeboPublisher:
    def __init__(self):
        rospy.init_node('gazebo_linkstate_controller')

        # ===== Publisher =====
        self.pub = rospy.Publisher(
            '/gazebo/set_link_state',
            LinkState,
            queue_size=10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # ===== Subscribers =====
        rospy.Subscriber('hand_pose', PoseStamped, self.hand_pose_cb)
        rospy.Subscriber('eye_pose', PoseStamped, self.eye_pose_cb)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb)

        # ===== internal state =====
        self.hand_current_pose = None

        # ===== gains =====
        self.kp_lin = 2.0
        self.kp_ang = 2.0

        # ===== target link name =====
        self.hand_link_name = 'hand::hand_link'  # ← 実際の link 名に合わせる
        self.eye_link_name  = 'eye::camera_link'
        self.world_frame = 'world'

    def publish_tf(self, pose, child_frame):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z
        tf_msg.transform.rotation.x = pose.orientation.x
        tf_msg.transform.rotation.y = pose.orientation.y
        tf_msg.transform.rotation.z = pose.orientation.z
        tf_msg.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(tf_msg)

    def link_states_cb(self, msg):
        if self.hand_link_name in msg.name:
            idx = msg.name.index(self.hand_link_name)
            self.hand_current_pose = msg.pose[idx]

    def hand_pose_cb(self, msg):
        if self.hand_current_pose is None:
            return

        state = LinkState()
        state.link_name = self.hand_link_name
        state.reference_frame = 'world'

        # ===== pose は現在値（必須・超重要）=====
        state.pose = self.hand_current_pose

        twist = Twist()

        # ===== 並進速度 =====
        dx = msg.pose.position.x - self.hand_current_pose.position.x
        dy = msg.pose.position.y - self.hand_current_pose.position.y
        dz = msg.pose.position.z - self.hand_current_pose.position.z

        twist.linear.x = self.kp_lin * dx
        twist.linear.y = self.kp_lin * dy
        twist.linear.z = self.kp_lin * dz

        # ===== 姿勢角速度（quaternion差分）=====
        q_t = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        q_c = [
            self.hand_current_pose.orientation.x,
            self.hand_current_pose.orientation.y,
            self.hand_current_pose.orientation.z,
            self.hand_current_pose.orientation.w
        ]

        q_e = tft.quaternion_multiply(
            q_t,
            tft.quaternion_inverse(q_c)
        )

        # shortest rotation
        if q_e[3] < 0.0:
            q_e = [-q_e[0], -q_e[1], -q_e[2], -q_e[3]]

        twist.angular.x = self.kp_ang * 2.0 * q_e[0]
        twist.angular.y = self.kp_ang * 2.0 * q_e[1]
        twist.angular.z = self.kp_ang * 2.0 * q_e[2]

        state.twist = twist
        self.pub.publish(state)
        self.publish_tf(self.hand_current_pose, self.hand_link_name)

    def eye_pose_cb(self, msg):
        # eye は pose 直接指定でも OK（attach しない前提）
        state = LinkState()
        state.link_name = self.eye_link_name
        state.reference_frame = 'world'
        state.pose = msg.pose
        state.twist = Twist()
        self.pub.publish(state)
        self.publish_tf(msg.pose, self.eye_link_name)


if __name__ == "__main__":
    try:
        PoseToGazeboPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
