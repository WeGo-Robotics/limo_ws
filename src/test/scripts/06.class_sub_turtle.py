#!/usr/bin/env python3
import rospy
from turtlesim.msg import Color

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/turtle1/color_sensor",Color,self.callback) #2. node 역할 설정

    def callback(self,msg): #3. subscriber - callback
        print(msg)

class_sub = Class_sub()
rospy.spin()