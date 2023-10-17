#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_node")  # ROS 1단계(필수) : 노드 이름
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # ROS 2단계(필수) : 노드 역할 - 퍼블리셔 설정
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # ROS 2단계(필수) : 노드 역할 - 서브스크라이버 설정
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션) : 퍼블리셔 - 주기 설정
        self.msg = Twist()  # 메세지 타입 설정 및 초기화

    def callback(self, msg):  # ROS 3단계(필수) : 서브스크라이버 - 콜백 함수 설정
        self.msg.linear.x = 1  # 메세지 항목(linear.x) - 데이터 값(1)으로 변경
        if msg.x >= 8:
            self.msg.linear.x = 0  # 메세지 항목(linear.x) - 데이터 값(0)으로 변경
        self.pub.publish(self.msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
        print(f"msg : {msg}")  # 출력 : 메세지
        print(f"msg.linear.x : {msg.linear.x}")  # 출력 : 메세지(linear.x)
        self.rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
