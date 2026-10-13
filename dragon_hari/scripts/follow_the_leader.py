#!/usr/bin/env python3

import math
import rospy
import tf2_ros

from aerial_robot_msgs.msg import FlightNav

from dragon_hari.motions.follow_the_leader.joint_player import (
    FollowTheLeaderJointPlayer,
)
from dragon_hari.motions.follow_the_leader.pose import (
    quaternion_to_yaw,
)


class FollowTheLeader:
    def __init__(self):
        """Follow-the-leader実行に必要なROS通信を準備する。"""

        self.joint_demo = FollowTheLeaderJointPlayer()

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        self.nav_publisher = rospy.Publisher(
            "/dragon/uav/nav",
            FlightNav,
            queue_size=1,
        )

    def wait_for_transform(
        self,
        target_frame,
        source_frame,
    ):
        """指定したTFを取得できるまで待つ。"""

        while not rospy.is_shutdown():
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0),
                )

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.loginfo(
                    "Waiting for transform: %s <- %s",
                    target_frame,
                    source_frame,
                )

                rospy.sleep(0.1)

        raise rospy.ROSInterruptException(
            "ROS shutdown while waiting for transform."
        )

    def wait_for_initial_state(self):
        """現在の機体状態を取得できるまで待つ。"""

        # 現在の関節角を取得
        self.current_joint_positions = (
                self.joint_demo.wait_for_joint_states(
                timeout=10.0
            )
        )

        # 現在のLink 1 poseを取得
        link1_transform = self.wait_for_transform(
            "world",
            "dragon/link1",
        )

        self.current_link1_x = (
            link1_transform.transform.translation.x
        )

        self.current_link1_y = (
            link1_transform.transform.translation.y
        )

        self.current_link1_yaw = quaternion_to_yaw(
            link1_transform.transform.rotation
        )

        # 現在のLink 4 poseを取得
        link4_transform = self.wait_for_transform(
            "world",
            "dragon/link4",
        )

        link4_x = (
            link4_transform.transform.translation.x
        )

        link4_y = (
            link4_transform.transform.translation.y
        )

        link4_yaw = quaternion_to_yaw(
            link4_transform.transform.rotation
        )

        # Link 4 originから+x方向へ0.474 m進んだ点をend tipとする
        self.current_end_tip_x = (
            link4_x
            + 0.474 * math.cos(link4_yaw)
        )

        self.current_end_tip_y = (
            link4_y
            + 0.474 * math.sin(link4_yaw)
        )

        rospy.loginfo("Initial end tip: x=%.3f, y=%.3f",
                      self.current_end_tip_x,
                      self.current_end_tip_y,)

    def build_trajectory(self, goal):
        """現在状態から目的地点までの基準軌跡を生成する."""
        pass

    def validate_trajectory(self, trajectory):
        """生成した軌跡が実行可能か確認する."""
        pass

    def prepare_for_playback(self, trajectory):
        """Follow-the-leader再生を開始できる状態へ移行する."""
        pass

    def execute_trajectory(self, trajectory, goal):
        """関節指令とCoG指令を同期して再生する."""
        pass

    def monitor_execution(self, goal):
        """目的地点への到達や異常状態を監視する."""
        pass

    def finish(self):
        """正常終了時の処理を行う."""
        pass


def read_goal():
    """world座標の目的地点x, yをユーザーから受け取る."""

    while not rospy.is_shutdown():
        try:
            goal_x, goal_y = map(
                float,
                input(
                    "Enter target x y [m]: "
                ).split(),
            )

            return goal_x, goal_y

        except ValueError:
            print(
                "Enter exactly 2 numbers: x y."
            )

    return None


def main():
    rospy.init_node(
        "follow_the_leader"
    )

    # 1. 目的地点を入力
    goal = read_goal()

    # 2. ノードを準備
    demo = FollowTheLeader()

    # 3. 現在状態を取得
    demo.wait_for_initial_state()

    # 4. 基準軌跡を生成
    trajectory = demo.build_trajectory(
        goal
    )

    # 5. 軌跡を検証
    demo.validate_trajectory(
        trajectory
    )

    # 6. 再生開始状態へ移行
    demo.prepare_for_playback(
        trajectory
    )

    # 7. Follow-the-leaderを実行
    demo.execute_trajectory(
        trajectory=trajectory,
        goal=goal,
    )

    # 8. 終了処理
    demo.finish()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass