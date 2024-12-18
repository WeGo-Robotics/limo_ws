#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from math import * 

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/scan",LaserScan,self.callback) #2. node 역할 설정
        self.lidar_msg = LaserScan()


    def callback(self,msg): #3. subscriber - callback
        print(msg)

class_sub = Class_sub()
rospy.spin()