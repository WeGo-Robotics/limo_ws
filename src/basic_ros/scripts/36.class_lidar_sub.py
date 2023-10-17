#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from math import *


class Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 클래스 2단계 : 클래스 초기화 및 초기 설정
        rospy.init_node("wego_sub_node")  # 1단계 : 노드의 이름
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)  # 2단계 : 노드의 역할
        laser_mag = LaserScan()
        self.laser_flag = False
        self.degrees = []

    def callback(self, msg):
        if self.laser_flag == False:
            self.degrees = [(msg.angle_min + msg.angle_increment * i) * 180 / pi for i, v in enumerate(msg.ranges)]

        self.laser_flag = True
        for index, value in enumerate(msg.ranges):
            if 0 < msg.ranges[index] < 0.30 and abs(self.degrees[index]) < 30:
                print(f"STOP:{self.degrees[index]}")


def main():  # 4 단계
    name = Name()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
