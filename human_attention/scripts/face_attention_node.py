#!/usr/bin/env python3

from std_msgs.msg import  Float32
import math
import os
import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
model_path = os.path.join(os.path.dirname(__file__), "../models/face_landmarker.task")

options = mp.tasks.vision.FaceLandmarkerOptions(base_options=mp.tasks.BaseOptions(model_asset_path=model_path), running_mode=mp.tasks.vision.RunningMode.IMAGE, num_faces=1, output_facial_transformation_matrixes=True)

landmarker = mp.tasks.vision.FaceLandmarker.create_from_options(options)

def image_cb(msg):
    # ROS画像をOpenCV画像に変換
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # BGRからRGBに変換
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # MediaPipe用の画像に変換
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    # 顔の特徴点を検出
    result = landmarker.detect(mp_image)

    if result.face_landmarks:
        matrix = result.facial_transformation_matrixes[0]
        # Yaw角（左右方向）
        yaw = math.degrees(math.atan2(matrix[0, 2], matrix[2, 2]))

        # Pitch角（上下方向）
        pitch = math.degrees(math.atan2(-matrix[1, 2], math.sqrt(matrix[1, 0]**2 + matrix[1, 1]**2)))
        rospy.loginfo_throttle(2, f"Yaw: {yaw:.1f} deg, Pitch: {pitch:.1f} deg")
        #rospy.loginfo_throttle(2, f"Matrix shape: {matrix.shape}")
        facing = abs(yaw) < 10 and abs(pitch) < 20

    else:
        facing = False
        # rospy.loginfo_throttle(2, "No face detected")

    rospy.loginfo_throttle(2,f"Facing camera: {int(facing)}")

    attention_pub.publish(Float32(data=float(facing)))

if __name__ == "__main__":
    rospy.init_node("face_attention")

    attention_pub = rospy.Publisher("/human/attention", Float32, queue_size=1)

    rospy.Subscriber("/usb_cam/image_raw", Image, image_cb, queue_size=1)

    rospy.spin()
