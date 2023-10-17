#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import *
import cv2
import numpy as np


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        self.sep_img_pub = rospy.Publisher("/camera/sep/image_raw/compressed", CompressedImage, queue_size=10)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)  # ROS 2단계(필수) : 노드 역할 - 퍼블리셔 설정
        self.rate = rospy.Rate(45)  # ROS 2-1단계(옵션) : 퍼블리셔 - 주기 설정
        self.cvbridge = CvBridge()  # 이미지 메세지를 OpenCV 이미지로 변경 시 사용되는 모듈(CvBridge) 설정
        self.img_msg = CompressedImage()  # 메세지 타입 설정 및 초기화
        self.cmd_msg = Twist()  # 메세지 타입 설정 및 초기화
        self.cam_flag = False  # 메세지 수신 확인을 위한 변수 설정

    def cam_CB(self, msg):  # ROS 3단계(필수) : 서브스크라이버 - 콜백 함수 설정
        if msg != -1:  # 메세지가 들어 왔을 시,
            self.img_msg = msg  # 메세지를 self.img_msg로 변경
            self.cam_flag = True  # 메세지 수신 확인 변수를 True로 변경
        else:
            self.cam_flag = False  # 메세지 수신 확인 변수를 False로 변경

    def sense(self):  # 센서 데이터 인지를 위한 함수(sense) 설정
        img_msg = self.img_msg  # 변수(img_msg)에 self.img_msg 값 저장
        cv_img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)  # 모듈(CvBridge)을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변경
        return cv_img  # 함수에서 값(cv_img) 리턴

    def think(self, cv_img):  # 데이터 처리 및 판단하는 함수(think) 설정
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)  # 모듈(openCV)를 사용하여 BGR 이미지(cv_img)를 HSV 이미지(hsv_img)로 색 공간 변경
        [width, height, channel] = hsv_img.shape  # HSV 이미지의 크기를 측정하여 너비(width), 높이(height), 채널(channel) 변수에 입력
        sep_img = hsv_img[width // 4 * 1 : width // 4 * 3, height // 4 * 1 : height // 4 * 3]  # HSV 이미지의 크기 중 일부를 잘라내어 변수(sep_img)에 저장
        avg_h = cv2.mean(sep_img)[0]  # 변수(sep_img)에서 색상(H) 채널의 평균값을 변수(avg_h)에 저장
        print(f"avg_h : {avg_h}")  # 출력 : 변수(avg_h)
        if avg_h < 30 or 150 < avg_h:  # 변수(avg_h)의 값이 30보다 작거나 150보다 큰 경우:
            motion = "STOP"  # 변수(motion)에 값("STOP") 저장
        elif 30 <= avg_h < 90:  # 변수(avg_h)의 값이 30보다 크고 90보다 작을 경우:
            motion = "SLOW"  # 변수(motion)에 값("SLOW") 저장
        else:  # 변수(avg_h)의 값이 90보다 크고 150보다 작을 경우:
            motion = "FAST"  # 변수(motion)에 값("FAST") 저장
        return sep_img, avg_h, motion  # 함수에서 값(hsv_img, avg_h, motion) 리턴

    def act(self, sep_img, motion):  # 판단된 데이터를 기반으로 제어하는 함수(act) 설정
        if motion == "STOP":  # 변수(motion)의 값이 "STOP" 인 경우:
            speed = 0  # 변수(speed)에 값(0)을 저장
        elif motion == "SLOW":  # 변수(motion)의 값이 "SLOW" 인 경우:
            speed = 0.5  # 변수(speed)에 값(0.5)을 저장
        else:  # 변수(motion)의 값이 "FAST" 인 경우:
            speed = 1.0  # 변수(speed)에 값(1.0)을 저장
        self.cmd_msg.linear.x = speed
        sep_img_mas = self.cvbridge.cv2_to_compressed_imgmsg(sep_img)
        self.sep_img_pub.publish(sep_img_mas)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        self.cmd_pub.publish(self.cmd_msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행

    def show(self, cv_img, sep_img):  # 이미지를 출력하는 함수(show) 설정
        cv2.namedWindow("cv_img", cv2.WINDOW_NORMAL)  # 창(cv_img) 설정
        cv2.imshow("cv_img", cv_img)  # 창(cv_img)에 변수(cv_img) 출력
        cv2.namedWindow("sep_img", cv2.WINDOW_NORMAL)  # 창(sep_img) 설정
        cv2.imshow("sep_img", sep_img)  # 창(sep_img)에 변수(sep_img) 출력
        cv2.waitKey(1)  # 창 종료 전 대기(0.001초)

    def func(self):  # 기능의 순서를 배치하는 함수(func) 설정
        if self.cam_flag == True:  # 변수(cam_flag)의 값이 True 인 경우
            cv_img = self.sense()  # 함수(sense)를 실행하여, 변수(cv_img)에 저장
            sep_img, avg_h, motion = self.think(cv_img)  # 함수(think)를 실행할 때, 변수(cv_img)를 입력하여, 변수(sep_img, avg_h, motion)로 출력
            self.act(sep_img, motion)  # 함수(act)를 실행할 때, 변수(sep_img, motion)를 입력
            self.show(cv_img, sep_img)  # 함수(show)를 실행할 때, 변수(cv_img, sep_img)를 입력
            print(f"Motion : {motion}({avg_h})")  # 출력 : 값(motion,avg_h)
        else:  # 변수(self.cam_flag)의 값이 False 인 경우
            pass  # 통과


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
