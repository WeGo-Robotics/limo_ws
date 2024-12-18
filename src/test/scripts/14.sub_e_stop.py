#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import *


class Class_sub:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node 이름 설정
        rospy.Subscriber("/scan", LaserScan, self.callback)  # 2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        self.lidar_msg = LaserScan()
        self.cmd_msg = Twist()

    def callback(self, msg):  # 3. subscriber - callback
        self.lidar_msg = msg
        degree_min = self.lidar_msg.angle_min * 180 / pi
        degree_max = self.lidar_msg.angle_max * 180 / pi
        degree_increment = self.lidar_msg.angle_increment * 180 / pi
        degrees = [degree_min + degree_increment * index for index, value in enumerate(self.lidar_msg.ranges)]
        self.obstacle = 0
        for index, value in enumerate(self.lidar_msg.ranges):
            if 0 < value < 0.5 and abs(degrees[index]) < 89:
                print(f"장애물:{degrees[index]}")
                self.cmd_msg.linear.x = 0
                self.pub.publish(self.cmd_msg)
            else:
                self.cmd_msg.linear.x = 0.3
                self.pub.publish(self.cmd_msg)
        # self.pub.publish(self.cmd_msg)


class_sub = Class_sub()
rospy.spin()
