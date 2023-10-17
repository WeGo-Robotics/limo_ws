#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 로봇 카메라로부터 이미지 데이터를 구독하고, 해당 이미지를 처리하여 움직임을 감지하고 제어하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 cv2, cv_bridge, numpy 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import *
import cv2
import numpy as np


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/camera/rgb/image_raw/image_raw/compressed" 토픽에서 압축된 이미지를 구독하고, 콜백 함수(cam_CB)를 호출합니다.
        rospy.Subscriber("/camera/rgb/image_raw/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/camera/gray/image_raw/compressed" 토픽에 압축된 이미지를 발행합니다.
        self.gray_pub = rospy.Publisher("/camera/gray/image_raw/compressed", CompressedImage, queue_size=10)  # ROS 2단계: 노드 역할 - 퍼블리셔 설정

        # 로봇을 제어하기 위한 퍼블리셔를 설정합니다.
        # "/cmd_vel" 토픽에 로봇 제어 명령을 발행합니다.
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)  # ROS 2단계(필수): 노드 역할 - 퍼블리셔 설정

        self.rate = rospy.Rate(45)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정
        self.cvbridge = CvBridge()  # 이미지 메세지를 OpenCV 이미지로 변경하기 위한 모듈(CvBridge) 설정
        self.img_msg = CompressedImage()  # 메세지 타입 설정 및 초기화
        self.cmd_msg = Twist()  # 메세지 타입 설정 및 초기화
        self.cam_flag = False  # 메세지 수신 확인을 위한 변수 설정

    # 카메라 이미지를 받아오는 콜백 함수(cam_CB) 설정
    def cam_CB(self, msg):  # ROS 3단계(필수): 서브스크라이버 - 콜백 함수 설정
        if msg != -1:  # 메세지가 유효한 경우
            self.img_msg = msg  # 메세지를 self.img_msg로 저장
            self.cam_flag = True  # 메세지 수신 확인 변수를 True로 설정
        else:
            self.cam_flag = False  # 메세지 수신 확인 변수를 False로 설정

    # 로봇의 센서 데이터를 인지하는 함수(sense) 설정
    def sense(self):
        img_msg = self.img_msg  # self.img_msg의 값을 img_msg 변수에 저장
        cv_img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)  # CvBridge 모듈을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환
        return cv_img  # 이미지를 반환

    # 이미지를 처리하고 로봇 동작을 결정하는 함수(think) 설정
    def think(self, cv_img):
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)  # OpenCV를 사용하여 BGR 이미지를 HSV 이미지로 변환
        [width, height, channel] = hsv_img.shape  # HSV 이미지의 크기를 측정하여 변수에 저장
        sep_img_left = hsv_img[0 : width // 2 * 1, :]  # HSV 이미지의 왼쪽 반 부분을 잘라내어 변수에 저장
        sep_img_right = hsv_img[width // 2 * 1 : :, :]  # HSV 이미지의 오른쪽 반 부분을 잘라내어 변수에 저장

        avg_v_left = cv2.mean(sep_img_left)[2]  # 왼쪽 부분의 명도(v) 채널의 평균값을 계산하여 변수에 저장
        avg_v_right = cv2.mean(sep_img_right)[2]  # 오른쪽 부분의 명도(v) 채널의 평균값을 계산하여 변수에 저장

        # 이미지 분석을 통해 로봇의 동작을 결정합니다.
        if avg_v_left < 64 and avg_v_right < 64:
            motion = "STOP"
            cv2.putText(cv_img, "STOP", (width // 7 * 4, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        elif avg_v_left > 192 and avg_v_right > 192:
            motion = "STRAIGHT"
            cv2.putText(cv_img, "STRAIGHT", (width // 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        else:
            if avg_v_left > avg_v_right:
                motion = "LEFT"
                cv2.putText(cv_img, "LEFT", (width // 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
            else:
                motion = "RIGHT"
                cv2.putText(cv_img, "RIGHT", (width // 7 * 5, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
        return hsv_img, motion, avg_v_left, avg_v_right  # 이미지, 동작 명령, 명도 값 반환

    # 로봇을 제어하는 함수(act) 설정
    def act(self, hsv_img, motion):
        if motion == "LEFT":
            steer = -1
        elif motion == "RIGHT":
            steer = 1
        else:
            steer = 0
        self.cmd_msg.linear.x = steer  # 로봇의 직선 운전 여부 설정
        hsv_img_mag = self.cvbridge.cv2_to_compressed_imgmsg(hsv_img)
        self.gray_pub.publish(hsv_img_mag)  # 압축된 이미지를 발행하여 화면 표시
        self.cmd_pub.publish(self.cmd_msg)  # 로봇 제어 명령을 발행
        self.rate.sleep()  # 주기적으로 반복 실행

    # 이미지를 화면에 표시하는 함수(show) 설정
    def show(self, cv_img, hsv_img):
        cv2.namedWindow("cv_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("cv_img", cv_img)  # 이미지를 화면에 표시
        cv2.namedWindow("hsv_img", cv2.WINDOW_NORMAL)  # OpenCV 이미지 창 설정
        cv2.imshow("hsv_img", hsv_img)  # 이미지를 화면에 표시
        cv2.waitKey(1)  # 사용자 입력 대기 (0.001초)

    # 메인 로직을 실행하는 함수(run) 설정
    def run(self):
        if self.cam_flag == True:  # 카메라 메시지가 수신된 경우
            cv_img = self.sense()  # 카메라 이미지를 인지
            hsv_img, motion, avg_v_left, avg_v_right = self.think(cv_img)  # 이미지 분석을 통해 동작 명령 및 명도 값 계산
            self.act(hsv_img, motion)  # 동작 명령을 기반으로 로봇 제어
            self.show(cv_img, hsv_img)  # 이미지를 화면에 표시
            print(f"Motion : {motion}, left:{avg_v_left}, right:{avg_v_right}")  # 동작 명령 및 명도 값 출력
        else:  # 카메라 메시지가 수신되지 않은 경우
            pass  # 아무 작업도 수행하지 않음


# 메인 함수 정의: ROS 노드를 실행하는 메인 함수
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()
    while not rospy.is_shutdown():  # ROS가 종료되지 않는 동안 계속 실행
        class_name.run()  # Class_Name 클래스의 run() 함수 실행


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
