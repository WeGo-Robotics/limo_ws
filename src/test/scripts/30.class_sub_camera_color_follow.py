#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import numpy as np

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_cb) #2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.cmd_msg = Twist()
        self.bridge = CvBridge() 
        
    def camera_cb(self,msg):
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv_img)
        
        y,x,channel = cv_img.shape
        red = 0
        yellow = 30
        green = 60
        cyan = 90
        blue = 120
        magenta = 150
        color = yellow
        if color==red :
            red1_lower=np.array([red,128,128])
            red1_upper=np.array([red+15,255,255])
            red1_filter = cv2.inRange(hsv_img,red1_lower,red1_upper)
            red2_lower=np.array([180-red,128,128])
            red2_upper=np.array([180,255,255])
            red2_filter = cv2.inRange(hsv_img,red2_lower,red2_upper)
            filter = cv2.bitwise_or(red1_filter,red2_filter)
        else :
            lower=np.array([color-15, 140,60])
            upper=np.array([color+15,255,255])
            filter = cv2.inRange(hsv_img,lower,upper)
        
        and_msg = cv2.bitwise_and(cv_img,cv_img,mask=filter)
        gray_img = cv2.cvtColor(and_msg,cv2.COLOR_BGR2GRAY)

        thresh_h = cv2.threshold(gray_img,40,255,cv2.THRESH_BINARY)
        struct = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        close_filter = cv2.morphologyEx(thresh_h,cv2.MORPH_CLOSE,struct)
        
        bin_img = np.zeros_like(filter)
        bin_img = gray_img
        bin_img[gray_img<30]=0
        hist_x = np.sum(bin_img,axis=0)
        hist_y = np.sum(bin_img,axis=1)
        print(1)
        

        contour,hierachy = cv2.findContours(close_filter,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        # contour_img = cv2.drawContours(cv_img,contour,-1,(0,255,255),3)
        try:
            cnt = contour[0]
            x,y,w,h = cv2.boundingRect(cnt)
            center_x = x+w//2
            center_y = y+h//2
            cv2.line(cv_img,(center_x,center_y),(center_x,center_y),(0,255,255),5)
            cv2.rectangle(cv_img,(x,y),(x+w,y+h),(0,255,0),3)
        except:
            center_x = x//2
            center_y = y//2

        center_index = x//2
        self.cmd_msg.angular.z = (center_x-center_index)*0.001
        self.pub.publish(self.cmd_msg)
        cv2.imshow("filter",filter)
        cv2.imshow("bin_img",bin_img)
        cv2.imshow("thresh_h",thresh_h)
        cv2.imshow("cv_img",cv_img)
        cv2.imshow("and_msg",and_msg)
        cv2.imshow("close_filter",close_filter)
        cv2.waitKey(1)
        
class_sub = Class_sub()
rospy.spin()