#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.client import Client

# 内部状態
s = None

# 内部状態を受信
def state_cb(msg):
    global s
    s = max(0.0, min(1.0, msg.data))

if __name__ == "__main__":
    rospy.init_node("gain_scheduler")
    # 内部状態を受信
    rospy.Subscriber("/human/internal_state", Float32, state_cb, queue_size=1)

    # DRAGONのXY位置制御に接続
    client = Client("/dragon/controller/xy", timeout=5)

    # PIDゲイン変更を有効化
    client.update_configuration({"pid_control_flag": True})

    rate = rospy.Rate(2) # 2 Hz
    last_kd = None

    while not rospy.is_shutdown():
        if s is not None:
            # 内部状態からDゲインを計算
            kd = 0.1 + 3.9 * s

            # ゲインが一定以上変化した場合だけ更新
            if last_kd is None or abs(kd - last_kd) >= 0.01:
                config = client.update_configuration({"d_gain": kd})
                last_kd = config["d_gain"]

                rospy.loginfo(f"s: {s:3f}, Kd: {kd: 3f}")

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
