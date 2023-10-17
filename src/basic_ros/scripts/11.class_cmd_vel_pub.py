#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_pub_node")  # ROS 1단계(필수) : 노드 이름
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # ROS 2단계(필수) : 노드 역할 - 퍼블리셔 설정
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션) : 퍼블리셔 - 주기 설정
        self.msg = Twist()  # 메세지 타입 설정 및 초기화

    def func(self):  # 클래스 3단계 : 함수 설정
        self.msg.linear.x = 1  # 메세지 항목(linear.x) - 데이터 값 변경
        self.pub.publish(self.msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        print(f"self.msg : {self.msg}")  # 출력 : 메세지
        print(f"self.msg.linear.x : {self.msg.linear.x}")  # 출력 : 메세지 항목(linear.x)
        self.rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    while not rospy.is_shutdown():
        class_name.func()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
