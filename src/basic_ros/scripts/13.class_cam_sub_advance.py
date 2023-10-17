#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import *
import cv2


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        rospy.Subscriber("/camera/rgb/image/raw/compressed", CompressedImage, self.callback)  # ROS 2단계(필수) : 노드 역할 - 서브스크라이버 설정
        self.cvbridge = CvBridge()  # 이미지 메세지를 OpenCV 이미지로 변경 시 사용되는 모듈(CvBridge) 설정

    def callback(self, msg):  # ROS 3단계(필수) : 서브스크라이버 - 콜백 함수 설정
        print(f"msg : {msg}")  # 출력 : 메세지
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)  # 모듈(CvBridge)을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변경
        print(img.shape)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("img", img)  # OpenCV 이미지 창 열기
        cv2.waitKey(1)  # OpenCV 이미지 창 대기


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
