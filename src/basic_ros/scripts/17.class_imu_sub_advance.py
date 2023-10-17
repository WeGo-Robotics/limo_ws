#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from math import *


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수) : 노드 이름
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계 : 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # ROS 3단계 : 서브스크라이버 - 콜백 함수 설정
        print(msg)  # 메세지 읽기
        print(f"msg.orientation:{msg.orientation}")  # 출력 : 메세지 항목(orientation)
        print(f"msg.linear_acceleration:{msg.linear_acceleration}")  # 출력 : 메세지 항목(linear_acceleration)
        print(f"msg.angular_velocity:{msg.angular_velocity}")  # 출력 : 메세지 항목(angular_velocity)

        roll_x_rad, pitch_y_rad, yaw_z_rad = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.w, msg.orientation.z)
        roll_x_degree = self.angle_to_degree(roll_x_rad)
        pitch_y_degree = self.angle_to_degree(pitch_y_rad)
        yaw_z_degree = self.angle_to_degree(yaw_z_rad)

        print(f"roll_x_degree:{roll_x_degree}")  # 출력 : 변수(roll_x_degree)
        print(f"pitch_y_degree:{pitch_y_degree}")  # 출력 : 변수(pitch_y_degree)
        print(f"yaw_z_degree:{yaw_z_degree}")  # 출력 : 변수(yaw_z_degree)

    def quaternion_to_euler(self, x, y, w, z):
        """
        쿼터니언을 오일러 각도(roll, pitch, yaw)로 변환
        롤은 라디안 단위로 x를 중심으로 회전합니다(시계 반대 방향)
        피치는 라디안 단위로 y를 중심으로 회전합니다(시계 반대 방향)
        yaw는 zin 라디안(반시계 방향)을 중심으로 회전합니다
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def angle_to_degree(self, angle):
        degree = angle * 180 / pi
        return degree


def main():  # 클래스 4단계 : 메인 함수
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 클래스 5단계 : 직접 실행 구문
    main()
