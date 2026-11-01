#!/usr/bin/env python3

import math
import time
import rospy
from std_msgs.msg import Float32

# パラメータ
s = 0.3
a = 0.0
tau = 3.0

# 注目状態を受信
def attention_cb(msg):
    global a
    a = msg.data


if __name__ == "__main__":
    rospy.init_node("internal_state")

    rospy.Subscriber("/human/attention", Float32, attention_cb, queue_size=1)

    state_pub = rospy.Publisher("/human/internal_state", Float32, queue_size=1)
    rate = rospy.Rate(10)
    last_time = time.monotonic()

    while not rospy.is_shutdown():
        now = time.monotonic()
        dt = now - last_time
        last_time = now

        # 内部状態の更新
        s = a + (s - a) * math.exp(-dt / tau)

        # 内部状態を配信
        state_pub.publish(Float32(data=s))

        rospy.loginfo_throttle(1, f"a: {a:1f}, s: {s:3f}")

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
