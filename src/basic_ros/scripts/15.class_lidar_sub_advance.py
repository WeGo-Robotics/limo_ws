#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from math import *


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        rospy.Subscriber("/scan", LaserScan, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계(필수) : 서브스크라이버 - 콜백 함수 설정
        print(f"msg : {msg}")  # 메세지 읽기
        print(f"msg.angle_min : {msg.angle_min}")  # 출력 : 메세지 항목(angle_min)
        print(f"msg.angle_max : {msg.angle_max}")  # 출력 : 메세지 항목(angle_max)
        print(f"msg.angle_increment : {msg.angle_increment}")  # 출력 : 메세지 항목(angle_increment)
        print(f"msg.range_min : {msg.range_min}")  # 출력 : 메세지 항목(range_min)
        print(f"msg.range_max : {msg.range_max}")  # 출력 : 메세지 항목(ranges_max)
        print(f"msg.ranges : {msg.ranges}")  # 출력 : 메세지 항목(ranges)

        degree_min = self.angle_to_degree(msg.angle_min)  # 변수(degree_min) - 데이터 값 쓰기
        degree_max = self.angle_to_degree(msg.angle_max)  # 변수(degree_max) - 데이터 값 쓰기
        degree_increment = self.angle_to_degree(msg.angle_increment)  # 변수(degree_increment) - 데이터 값 쓰기
        degrees = self.make_degree_array(degree_min, degree_increment, msg.ranges)  # 변수(degree) - 데이터 값 쓰기

        print(f"degree_min:{degree_min}")  # 출력 : 변수(degree_min)
        print(f"degree_max:{degree_max}")  # 출력 : 변수(degree_max)
        print(f"degree_increment:{degree_increment}")  # 출력 : 변수(degree_increment)
        print(f"degrees:{degrees}")  # 출력 : 변수(degree)

    def angle_to_degree(self, angle):
        degree = angle * 180 / pi
        return degree

    def make_degree_array(self, degree_min, degree_increment, ranges):
        degrees = [degree_min + (index * degree_increment) for index, value in enumerate(ranges)]
        return degrees


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
