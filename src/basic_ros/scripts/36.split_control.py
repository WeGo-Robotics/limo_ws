#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from math import *
import os
import csv


# 클래스 생성
class NAME:
    # 초기화 및 초기 설정
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        rospy.Subscriber("/imu", Imu, self.imu_CB)
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration.x
        self.ctrl_msg = Twist()
        rospy.Timer(rospy.Duration(1), self.ctrl)
        self.speed = 0

    # 함수 설정
    def imu_CB(self, msg):
        self.imu_msg = msg

    def ctrl(self, _):
        self.speed = self.imu_msg.linear_acceleration.x
        self.ctrl_msg.linear.x = self.speed
        self.ctrl_pub.publish(self.ctrl_msg)


def main():
    name = NAME()
    rospy.spin()


if __name__ == "__main__":
    main()
