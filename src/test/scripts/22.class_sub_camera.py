#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_cb) #2. node 역할 설정

    def camera_cb(self,msg):
        print(msg)

class_sub = Class_sub()
rospy.spin()