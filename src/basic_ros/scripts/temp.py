#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import math


class ImageProcessor:
    def __init__(self):
        # CvBridge 인스턴스를 초기화하여 ROS 이미지를 OpenCV 이미지로 변환합니다.
        self.bridge = CvBridge()

        # ROS 노드를 초기화하고 노드 이름을 'image_processor_node'로 설정합니다.
        rospy.init_node("image_processor_node", anonymous=True)

        # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 Subscriber하고,
        # 새 메시지가 수신되면 self.image_callback 메서드를 호출합니다.
        # rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback)

        # 'cmd_vel' 토픽에 Twist 메시지를 발행할 퍼블리셔를 초기화합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Create the trackbars for the HSV ranges
        self.win_name = "image"

        self.L_H_Value = 0
        self.L_S_Value = 0
        self.L_V_Value = 0
        self.U_H_Value = 255
        self.U_S_Value = 255
        self.U_V_Value = 255
        self.create_trackbar_init()

    def create_trackbar_init(self):
        # cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        a = cv2.imread("lane.jpg")
        cv2.imshow(self.win_name, a)
        cv2.waitKey(1)
        #  Create a named window for the trackbar GUI
        # cv2.create
        # Create trackbars for the HSV range
        cv2.createTrackbar("LH", self.win_name, 0, 179, self.hsv_track)
        cv2.createTrackbar("LS", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("LV", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("UH", self.win_name, 179, 179, self.hsv_track)
        cv2.createTrackbar("US", self.win_name, 255, 255, self.hsv_track)
        cv2.createTrackbar("UV", self.win_name, 255, 255, self.hsv_track)

    def hsv_track(self, value):
        # Get the values of the trackbars
        self.L_H_Value = cv2.getTrackbarPos("LH", self.win_name)
        self.L_S_Value = cv2.getTrackbarPos("LS", self.win_name)
        self.L_V_Value = cv2.getTrackbarPos("LV", self.win_name)
        self.U_H_Value = cv2.getTrackbarPos("UH", self.win_name)
        self.U_S_Value = cv2.getTrackbarPos("US", self.win_name)
        self.U_V_Value = cv2.getTrackbarPos("UV", self.win_name)

    def image_callback(self, msg):
        # CompressedImage 메시지를 OpenCV 이미지로 변환합니다.
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # Convert the BGR image to HSV
        cvt_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the BGR range using the trackbar values
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        upper = np.array([85, self.U_S_Value, self.U_V_Value])

        # Create a mask using the HSV range
        mask = cv2.inRange(cvt_hsv, lower, upper)

        # Apply the mask to the original image to obtain the final result
        rst = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow(self.win_name, image)
        print(self.L_H_Value)
        print(self.L_S_Value)
        print(self.L_V_Value)
        print(self.U_H_Value)
        print(self.U_S_Value)
        print(self.U_V_Value)
        cv2.waitKey(1)

    def main(self):
        try:
            # ROS 노드를 실행합니다.
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()


def run():
    # ImageProcessor 인스턴스를 생성하고 run 메서드를 호출하여 프로그램을 실행합니다.
    image_processor = ImageProcessor()
    image_processor.main()


if __name__ == "__main__":
    run()
