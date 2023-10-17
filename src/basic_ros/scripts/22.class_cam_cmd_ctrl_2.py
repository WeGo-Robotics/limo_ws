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
        rospy.Subscriber("/camera/rgb/image_raw/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정
        self.gray_pub = rospy.Publisher("/camera/gray/image_raw/compressed", CompressedImage, queue_size=10)  # ROS 2단계 : 노드 역할 - 퍼블리셔 설정
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
        sep_img_left = hsv_img[0 : width // 2 * 1, :]  # HSV 이미지의 크기 중 일부를 잘라내어 변수(sep_img_left)에 저장
        sep_img_right = hsv_img[width // 2 * 1 : :, :]  # HSV 이미지의 크기 중 일부를 잘라내어 변수(sep_img_right)에 저장

        avg_v_left = cv2.mean(sep_img_left)[2]  # 변수(sep_img_left)에 명도(v) 채널의 평균값을 변수(avg_v_left)에 저장
        avg_v_right = cv2.mean(sep_img_right)[2]  # 변수(sep_img_right)에 명도(v) 채널의 평균값을 변수(avg_v_right)에 저장

        if avg_v_left < 64 and avg_v_right < 64:  # 변수(avg_v_left)의 값이 64보다 작고 변수(avg_v_right)의 값이 64보다 작은 경우:
            motion = "STOP"  # 변수(motion)에 값("STOP") 저장
            cv2.putText(cv_img, "STOP", (width // 7 * 4, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        elif avg_v_left > 192 and avg_v_right > 192:  # 변수(avg_v_left)의 값이 192보다 크고 변수(avg_v_right)의 값이 192보다 큰 경우:
            motion = "STRAIGHT"  # 변수(motion)에 값("STRAIGHT") 저장
            cv2.putText(cv_img, "STRAIGHT", (width // 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        else:
            if avg_v_left > avg_v_right:  # 변수(avg_v_left)의 값이 변수(avg_v_right)의 값 보다 큰 경우:
                motion = "LEFT"  # 변수(motion)에 값("LEFT") 저장
                cv2.putText(cv_img, "LEFT", (width // 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
            else:  # 변수(avg_v_left)의 값이 변수(avg_v_right)의 값 보다 작은 경우:
                motion = "RIGHT"  # 변수(motion)에 값("RIGHT") 저장
                cv2.putText(cv_img, "RIGHT", (width // 7 * 5, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        return hsv_img, motion  # 함수에서 값(hsv_img, motion) 리턴

    def act(self, hsv_img, motion):  # 판단된 데이터를 기반으로 제어하는 함수(act) 설정
        if motion == "LEFT":  # 변수(motion)에 값이 "LEFT"일 경우
            steer = -1  # 변수(steer)에 값(-1)을 저장
        elif motion == "RIGHT":  # 변수(motion)에 값이 "RIGHT"일 경우
            steer = 1  # 변수(steer)에 값(1)을 저장
        else:
            steer = 0  # 변수(steer)에 값(0)을 저장
        self.cmd_msg.linear.x = steer  # 변수(steer)에 값(0)을 저장
        hsv_img_mag = self.cvbridge.cv2_to_compressed_imgmsg(hsv_img)
        self.gray_pub.publish(hsv_img_mag)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        self.cmd_pub.publish(self.cmd_msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행

    def show(self, cv_img, hsv_img):
        cv2.namedWindow("cv_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("cv_img", cv_img)  # 창(cv_img)에 변수(cv_img) 출력
        cv2.namedWindow("hsv_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("hsv_img", hsv_img)  # 창(hsv_img)에 변수(hsv_img) 출력
        cv2.waitKey(1)  # 창 종료 전 대기(0.001초)

    def func(self):
        if self.cam_flag == True:
            cv_img = self.sense()
            hsv_img, motion = self.think(cv_img)
            self.act(hsv_img, motion)
            self.show(cv_img, hsv_img)
            print(f"Motion : {motion}")
        else:
            pass


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
