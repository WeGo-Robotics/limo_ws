#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import *
import cv2


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        self.pub = rospy.Publisher("/camera/grayscale/image_raw/compressed", CompressedImage, queue_size=10)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
        rospy.Subscriber("/camera/rgb/image/raw/compressed", CompressedImage, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        self.rate = rospy.Rate(30)  # ROS 2-1단계(옵션) : 퍼블리셔 - 주기 설정
        self.cvbridge = CvBridge()  # 이미지 메세지를 OpenCV 이미지로 변경 시 사용되는 모듈(CvBridge) 설정
        self.gray_img_msg = CompressedImage()

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(f"msg : {msg}")  # 출력 : 메세지
        color_img = self.cvbridge.compressed_imgmsg_to_cv2(msg)  # 모듈(CvBridge)을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변경
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)  # 모듈(openCV)을 사용하여, openCV 컬러 이미지를 그레이 스케일 이미지로 변경
        self.gray_img_msg = self.cvbridge.cv2_to_compressed_imgmsg(gray_img)  # 모듈(CvBridge)을 사용하여 OpenCV 이미지를 이미지 메세지(CompressedImage)를  변경
        self.pub.publish(self.gray_img_msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행

        cv2.namedWindow("color_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("color_img", color_img)  # OpenCV 이미지 창 열기
        cv2.namedWindow("gray_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("gray_img", gray_img)  # OpenCV 이미지 창 열기
        cv2.waitKey(1)  # OpenCV 이미지 창 대기


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
