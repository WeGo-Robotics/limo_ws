#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 레이저 스캔(LaserScan) 데이터를 구독하고 해당 데이터를 분석하여 각도(degree)를 계산하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형 및 수학 함수를 가져옵니다.
import rospy
from sensor_msgs.msg import LaserScan
from math import *


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/scan" 토픽에서 LaserScan 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/scan", LaserScan, self.callback)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # LaserScan 메시지의 다양한 속성을 출력합니다.
        print(f"msg : {msg}")  # 메시지 읽기
        print(f"msg.angle_min : {msg.angle_min}")  # 출력: 메시지 항목(angle_min)
        print(f"msg.angle_max : {msg.angle_max}")  # 출력: 메시지 항목(angle_max)
        print(f"msg.angle_increment : {msg.angle_increment}")  # 출력: 메시지 항목(angle_increment)
        print(f"msg.range_min : {msg.range_min}")  # 출력: 메시지 항목(range_min)
        print(f"msg.range_max : {msg.range_max}")  # 출력: 메시지 항목(ranges_max)
        print(f"msg.ranges : {msg.ranges}")  # 출력: 메시지 항목(ranges)

        degree_min = self.angle_to_degree(msg.angle_min)  # 함수 호출: 라디안 각도를 도(degree)로 변환
        degree_max = self.angle_to_degree(msg.angle_max)  # 함수 호출: 라디안 각도를 도(degree)로 변환
        degree_increment = self.angle_to_degree(msg.angle_increment)  # 함수 호출: 라디안 각도를 도(degree)로 변환

        # 함수 호출: 각도 배열 생성
        degrees = self.make_degree_array(degree_min, degree_increment, msg.ranges)

        # 출력: 변환된 각도 및 각도 배열
        print(f"degree_min:{degree_min}")
        print(f"degree_max:{degree_max}")
        print(f"degree_increment:{degree_increment}")
        print(f"degrees:{degrees}")

    def angle_to_degree(self, angle):
        # 라디안(radian) 각도를 도(degree)로 변환
        degree = angle * 180 / pi
        return degree

    def make_degree_array(self, degree_min, degree_increment, ranges):
        # 각도 배열 생성
        degrees = [degree_min + (index * degree_increment) for index, value in enumerate(ranges)]
        return degrees


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
