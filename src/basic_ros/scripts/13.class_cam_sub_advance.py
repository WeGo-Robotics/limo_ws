#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 이미지 데이터를 구독하고 이를 OpenCV 이미지로 변환하여 표시하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형 및 cv_bridge 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import *
import cv2


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/camera/rgb/image/raw/compressed" 토픽에서 CompressedImage 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/camera/rgb/image/raw/compressed", CompressedImage, self.callback)  # ROS 2단계(필수): 서브스크라이버 설정

        # CvBridge 모듈을 사용하여 이미지 메시지(CompressedImage)를 OpenCV 이미지로 변환할 준비를 합니다.
        self.cvbridge = CvBridge()

    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # 이미지 메시지를 OpenCV 이미지로 변환합니다.
        img = self.cvbridge.compressed_imgmsg_to_cv2(msg)

        # 이미지를 출력하기 위해 OpenCV 윈도우 창을 설정합니다.
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)  # OpenCV 이미지 창을 엽니다.
        cv2.waitKey(1)  # OpenCV 이미지 창을 대기합니다.


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
