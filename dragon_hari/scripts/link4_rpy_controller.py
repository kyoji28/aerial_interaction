#!/usr/bin/env python3
"""Link 4の目標RPYを、Baselinkの姿勢指令に変換するROS 1ノード。"""

import math

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)


class Link4RPYController:
    def __init__(self):
        self.target_rpy = None  # world基準、rad
        robot_ns = rospy.get_param("~robot_ns", "dragon").strip("/")
        self.baselink_frame = "/".join(filter(None, (robot_ns, "fc")))
        self.link4_frame = "/".join(filter(None, (robot_ns, "link4")))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            "/dragon/target_rotation_motion", Odometry, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_command)  # 20 Hz

    def publish_command(self, _event):
        if self.target_rpy is None:
            return

        try:
            # fcはBaselink（Link 2）。Link 4の相対姿勢をTFから取得する。
            transform = self.tf_buffer.lookup_transform(
                self.baselink_frame, self.link4_frame, rospy.Time(0)
            )
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(5.0, "Link 4のTFを取得できません: %s", exc)
            return

        rotation = transform.transform.rotation
        q_b4 = [rotation.x, rotation.y, rotation.z, rotation.w]
        q_w4 = quaternion_from_euler(*self.target_rpy)
        q_wb = quaternion_multiply(q_w4, quaternion_inverse(q_b4))

        command = Odometry()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "baselink"
        command.pose.pose.orientation.x = float(q_wb[0])
        command.pose.pose.orientation.y = float(q_wb[1])
        command.pose.pose.orientation.z = float(q_wb[2])
        command.pose.pose.orientation.w = float(q_wb[3])
        # 目標角速度は0（低速の姿勢変更・関節動作を想定）。
        self.publisher.publish(command)

    def print_current_rpy(self):
        """現在のLink 4の姿勢をworld基準の角度[deg]で表示する。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", self.link4_frame, rospy.Time(0)
            )
        except tf2_ros.TransformException as exc:
            print("現在のLink 4姿勢を取得できません: {}".format(exc))
            return

        q = transform.transform.rotation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        angles = [math.degrees(v) for v in rpy]
        print("現在のLink 4 RPY [deg]: ({:+.1f}, {:+.1f}, {:+.1f})".format(*angles))

    def keyboard_loop(self):
        print("使用TF: {} → {}".format(self.baselink_frame, self.link4_frame))
        print("Link 4の目標Roll Pitch Yawをスペース区切りで入力してください（度）。")
        print("例: 0 0 30  /  10 20 30  /  0 0 0（戻す）")
        print("s（現在のRPYを表示） / q（終了）")
        self.print_current_rpy()

        while not rospy.is_shutdown():
            try:
                line = input("RPY [deg] > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if line.lower() in ("q", "quit", "exit"):
                break
            if line.lower() in ("s", "status", ""):
                self.print_current_rpy()
                continue

            try:
                angles = [float(v) for v in line.replace(",", " ").split()]
                if len(angles) != 3 or not all(math.isfinite(v) for v in angles):
                    raise ValueError
            except ValueError:
                print("Roll Pitch Yawの3つの数値を入力してください。例: 0 0 30")
                continue

            self.target_rpy = tuple(math.radians(v) for v in angles)
            print("Link 4の目標RPYを設定: ({:g}, {:g}, {:g}) deg".format(*angles))
            self.print_current_rpy()


if __name__ == "__main__":
    rospy.init_node("link4_rpy_controller")
    controller = Link4RPYController()
    try:
        controller.keyboard_loop()
    finally:
        rospy.signal_shutdown("キーボード入力を終了しました")
