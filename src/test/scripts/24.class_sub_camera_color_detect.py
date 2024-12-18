#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_cb) #2. node 역할 설정
        self.bridge = CvBridge() 
    def camera_cb(self,msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
        white_lower = np.array([0,0,120])
        white_upper = np.array([179,100,255])
        white_filter = cv2.inRange(hsv_img,white_lower,white_upper)
        yellow_lower = np.array([15,30,80])
        yellow_upper = np.array([45,255,255])
        yellow_filter = cv2.inRange(hsv_img,yellow_lower,yellow_upper)
        combine_filter = cv2.bitwise_or(white_filter,yellow_filter)
        
        and_img = cv2.bitwise_and(cv_img,cv_img,mask=combine_filter)
        cv2.imshow("cv_img",cv_img)
        
        cv2.imshow("and_img",and_img)
        cv2.imshow("white_filter",white_filter)
        cv2.imshow("yellow_filter",yellow_filter)
        cv2.waitKey(1)
        
class_sub = Class_sub()
rospy.spin()