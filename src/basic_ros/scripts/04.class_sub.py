#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        self.sub = rospy.Subscriber("/counter", Int32, self.callback)  # ROS 2단계(필수) : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계(필수) : 서브스크라이버 - 콜백 함수 설정
        print(f"msg:{msg}")  # 출력 : 메세지
        print(f"msg.data:{msg.data}")  # 출력 : 메세지 항목(데이터)


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
