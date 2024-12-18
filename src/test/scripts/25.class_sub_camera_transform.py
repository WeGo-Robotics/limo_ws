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
        y,x,channel = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)

        white_lower = np.array([0,0,150])
        white_upper = np.array([179,20,255])
        white_filter = cv2.inRange(hsv_img,white_lower,white_upper)

        yellow_lower = np.array([15,60,80])
        yellow_upper = np.array([45,255,255])
        yellow_filter = cv2.inRange(hsv_img,yellow_lower,yellow_upper)
        combine_filter = cv2.bitwise_or(white_filter,yellow_filter)
        
        and_img = cv2.bitwise_and(cv_img,cv_img,mask=combine_filter)
        
        margin_x = 250
        margin_y = 300

        src_pt1 = (0,y)
        src_pt2 = (margin_x,margin_y)
        src_pt3 = (x-margin_x,margin_y)
        src_pt4 = (x,y)
        src_pts = np.float32([src_pt1,src_pt2,src_pt3,src_pt4])

        dst_margin_x = 120

        dst_pt1 = (dst_margin_x,y)
        dst_pt2 = (dst_margin_x,0)
        dst_pt3 = (x-dst_margin_x,0)
        dst_pt4 = (x-dst_margin_x,y)
        dst_pts = np.float32([dst_pt1,dst_pt2,dst_pt3,dst_pt4])
        
        matrix = cv2.getPerspectiveTransform(src_pts,dst_pts)
        warp_img = cv2.warpPerspective(and_img,matrix,(x,y))
    
        cv2.circle(cv_img,src_pt1,20,(255,0,0),-1)
        cv2.circle(cv_img,src_pt2,20,(0,255,0),-1)
        cv2.circle(cv_img,src_pt3,20,(0,0,255),-1)
        cv2.circle(cv_img,src_pt4,20,(0,255,255),-1)
    
        cv2.imshow("cv_img",cv_img)
        cv2.imshow("and_img",and_img)
        # cv2.imshow("combine_filter",combine_filter)
        cv2.imshow("warp_img",warp_img)
        cv2.waitKey(1)
        
class_sub = Class_sub()
rospy.spin()