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
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/camera/rgb/image_raw/compressed" 토픽에서 CompressedImage 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.cam_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/camera/sep/image_raw/compressed" 토픽에 CompressedImage 메시지를 발행합니다.
        self.sep_img_pub = rospy.Publisher("/camera/sep/image_raw/compressed", CompressedImage, queue_size=10)  # ROS 2단계: 노드 역할 - 퍼블리셔 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/cmd_vel" 토픽에 Twist 메시지를 발행합니다.
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)  # ROS 2단계(필수): 노드 역할 - 퍼블리셔 설정

        self.rate = rospy.Rate(45)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정

        self.cvbridge = CvBridge()  # 이미지 메세지를 OpenCV 이미지로 변경 시 사용되는 모듈(CvBridge) 설정

        self.img_msg = CompressedImage()  # 메세지 타입 설정 및 초기화
        self.cmd_msg = Twist()  # 메세지 타입 설정 및 초기화

        self.cam_flag = False  # 이미지 메세지 수신 확인을 위한 변수 설정

    def cam_CB(self, msg):  # 3단계(필수): 서브스크라이버 - 콜백 함수 설정
        if msg != -1:  # 메세지가 들어 왔을 때:
            self.img_msg = msg  # 메세지를 self.img_msg로 저장
            self.cam_flag = True  # 이미지 메세지 수신 확인 변수를 True로 저장
        else:  # 메세지가 없을 때:
            self.cam_flag = False  # 이미지 메세지 수신 확인 변수를 False로 저장

    def sense(self):  # 센서 데이터 인지를 위한 함수(sense) 설정
        img_msg = self.img_msg  # 이미지 메세지를 변수(img_msg)에 저장
        cv_img = self.cvbridge.compressed_imgmsg_to_cv2(img_msg)  # 모듈(CvBridge)을 사용하여 이미지 메세지(CompressedImage)를 OpenCV 이미지로 변환
        return cv_img  # 함수에서 변환된 이미지(cv_img)를 반환

    def think(self, cv_img):  # 데이터 처리 및 판단하는 함수(think) 설정
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)  # OpenCV를 사용하여 BGR 이미지(cv_img)를 HSV 이미지(hsv_img)로 변환
        [width, height, channel] = hsv_img.shape  # HSV 이미지의 크기를 측정하여 너비(width), 높이(height), 채널(channel) 변수에 저장
        sep_img = hsv_img[width // 4 * 1 : width // 4 * 3, height // 4 * 1 : height // 4 * 3]  # HSV 이미지의 일부를 자르고 변수(sep_img)에 저장
        avg_h = cv2.mean(sep_img)[0]  # 변수(sep_img)에서 색상(H) 채널의 평균값을 변수(avg_h)에 저장
        print(f"avg_h : {avg_h}")  # 출력: 변수(avg_h)

        if avg_h < 30 or 150 < avg_h:  # 변수(avg_h)가 30보다 작거나 150보다 큰 경우:
            motion = "STOP"  # 변수(motion)에 "STOP" 저장
        elif 30 <= avg_h < 90:  # 변수(avg_h)가 30보다 크거나 같고 90보다 작은 경우:
            motion = "SLOW"  # 변수(motion)에 "SLOW" 저장
        else:  # 변수(avg_h)가 90보다 크고 150보다 작은 경우:
            motion = "FAST"  # 변수(motion)에 "FAST" 저장

        return sep_img, avg_h, motion  # 함수에서 변수(hsv_img, avg_h, motion)를 반환

    def act(self, sep_img, motion):  # 판단된 데이터를 기반으로 제어하는 함수(act) 설정
        if motion == "STOP":  # 변수(motion)이 "STOP"인 경우:
            speed = 0  # 변수(speed)에 0 저장
        elif motion == "SLOW":  # 변수(motion)이 "SLOW"인 경우:
            speed = 0.5  # 변수(speed)에 0.5 저장
        else:  # 변수(motion)이 "FAST"인 경우:
            speed = 1.0  # 변수(speed)에 1.0 저장

        self.cmd_msg.linear.x = speed  # Twist 메시지의 선속도(linear.x)에 변수(speed) 저장

        sep_img_msg = self.cvbridge.cv2_to_compressed_imgmsg(sep_img)  # OpenCV 이미지(sep_img)를 이미지 메세지(CompressedImage)로 변환
        self.sep_img_pub.publish(sep_img_msg)  # "/camera/sep/image_raw/compressed" 토픽에 이미지 메세지를 발행

        self.cmd_pub.publish(self.cmd_msg)  # "/cmd_vel" 토픽에 Twist 메시지를 발행

        self.rate.sleep()  # 주기를 맞추기 위해 대기

    def show(self, cv_img, sep_img):  # 이미지를 출력하는 함수(show) 설정
        cv2.namedWindow("cv_img", cv2.WINDOW_NORMAL)  # OpenCV 창 "cv_img" 설정
        cv2.imshow("cv_img", cv_img)  # "cv_img" 창에 OpenCV 이미지(cv_img)를 출력
        cv2.namedWindow("sep_img", cv2.WINDOW_NORMAL)  # OpenCV 창 "sep_img" 설정
        cv2.imshow("sep_img", sep_img)  # "sep_img" 창에 OpenCV 이미지(sep_img)를 출력
        cv2.waitKey(1)  # 1 밀리초 동안 창을 대기

    def run(self):  # 기능의 순서를 배치하는 함수(run) 설정
        if self.cam_flag == True:  # 이미지 메세지 수신 확인 변수(cam_flag)가 True인 경우:
            cv_img = self.sense()  # 이미지를 sense() 함수를 통해 가져와 변수(cv_img)에 저장
            sep_img, avg_h, motion = self.think(cv_img)  # think() 함수를 실행하여 변수(sep_img, avg_h, motion)에 저장
            self.act(sep_img, motion)  # act() 함수를 실행하여 로봇 동작을 수행
            self.show(cv_img, sep_img)  # show() 함수를 실행하여 이미지를 화면에 출력
            print(f"Motion : {motion}({avg_h})")  # 출력: 로봇의 동작(motion)과 평균 H값(avg_h)
        else:  # 이미지 메세지 수신 확인 변수(cam_flag)가 False인 경우:
            pass  # 아무 작업도 수행하지 않음


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.
    while not rospy.is_shutdown():  # ROS가 종료되지 않는 동안 반복 실행
        class_name.run()  # run() 함수를 실행합니다.


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
