#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_cb) #2. node 역할 설정
        self.bridge = CvBridge() 
    def camera_cb(self,msg):
        # print(msg)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("cv_img",cv_img)
        cv2.waitKey(1)
        
class_sub = Class_sub()
rospy.spin()