#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
from math import *

class Camera_ctrl:
    def __init__(self):
        rospy.init_node("wego_node")
        
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_CB)
        self.ctrl_pub = rospy.Publisher("cmd_vel",Twist,queue_size=3)
        self.camera_msg = CompressedImage()
        self.bridge = CvBridge()
        
    def camera_CB(self,msg):
        self.camera_msg = msg
        cv_img = self.bridge.compressed_imgmsg_to_cv2(self.camera_msg)
        
        cv_img = cv2.imread("/home/wego/basic_ros_ws/src/basic_ros/scripts/lane.jpg",cv2.IMREAD_COLOR) # 우분투 버전

        gray_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
        edge_img = cv2.Canny(gray_img, 50, 150)
        cv2.imshow("edge_img",edge_img)
        src = np.float32([[210,639],[200,190],[640-200,190],[640-210,639]])
        dst = np.float32([[210,639],[210,0],[640-210,0],[640-210,639]])

        #src = np.float([x1,img.shape(0)],[x2,y2],[img.shape(1)-x2,y2],[img.shape(1)-x1,img.shape(0)])
        #src = np.float([x1,img.shape(0)],[x2,0],[img.shape(1)-x2,0],[img.shape(1)-x1,img.shape(0)])

        matrix = cv2.getPerspectiveTransform(src,dst)
        edge_warp_img = cv2.warpPerspective(edge_img,matrix,(edge_img.shape[1],edge_img.shape[0]))
        cv_warp_img = cv2.warpPerspective(cv_img,matrix,(edge_img.shape[1],edge_img.shape[0]))
        lines = cv2.HoughLinesP(edge_warp_img,1,np.pi/180,30,minLineLength=100,maxLineGap=5)
        if lines is not None: # 라인 정보를 받았으면
            for i in range(lines.shape[0]):
                pt1 = (lines[i][0][0], lines[i][0][1]) # 시작점 좌표 x,y
                pt2 = (lines[i][0][2], lines[i][0][3]) # 끝점 좌표, 가운데는 무조건 0
                cv2.line(cv_warp_img, pt1, pt2,(0, 0, 255), 2, cv2.LINE_AA)
        cv2.imshow("cv_img",cv_img)
        cv2.imshow("edge_warp_img",edge_warp_img)
        cv2.imshow("cv_warp_img",cv_warp_img)
        cv2.waitKey(1)
    
def main():
    camera_ctrl=Camera_ctrl()
    rospy.spin()

if __name__ == "__main__":
    main()

