#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import *


class Class_Name:  # 클래스 1단계 : 클래스 이름
    def __init__(self):  # 2 단계
        rospy.init_node("wego_sub_node")  # 1단계 : 노드의 이름
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)  # 2단계 : 노드의 역할
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2단계 : 노드의 역할
        laser_mag = LaserScan()
        self.ctrl_msg = Twist()
        self.laser_flag = False
        self.degrees = []

    def callback(self, msg):
        object_degrees = []
        if self.laser_flag == False:
            self.degrees = [(msg.angle_min + msg.angle_increment * i) * 180 / pi for i, v in enumerate(msg.ranges)]
            self.laser_flag = True

        for index, value in enumerate(msg.ranges):
            if 0 < msg.ranges[index] < 0.30 and abs(self.degrees[index]) < 30:
                object_degrees.append(self.degrees[index])
            else:
                pass

        if len(object_degrees) > 30:
            print(f"STOP:{object_degrees}")
            speed = 0
        else:
            print("GO")
            speed = 1

        self.ctrl_msg.linear.x = speed
        self.pub.publish(self.ctrl_msg)


def main():  # 4 단계
    class_name = Class_Name()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
