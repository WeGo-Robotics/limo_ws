#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan,CompressedImage,Imu
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
from math import *

class Total_data:
    def __init__(self):
        rospy.init_node('wego_node')
        rospy.Subscriber("/scan",LaserScan,self.laser_CB)
        rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,self.camera_CB)
        rospy.Subscriber("/imu",Imu,self.imu_CB)
        self.bridge = CvBridge()
        self.ctrl_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.laser_msg = LaserScan()
        self.camera_msg = CompressedImage()
        self.cv_img = []
        self.imu_msg = Imu()
        
        self.main()

    def laser_CB(self,msg):
        if msg != -1:
            self.laser_msg = msg
    
    def camera_CB(self,msg):
        if msg != -1:
            self.camera_msg = msg
            self.cv_img = self.bridge.compressed_imgmsg_to_cv2(self.camera_msg)
 
            cv2.imshow("cv_img",self.cv_img)
            cv2.waitKey(1)    

    def imu_CB(self,msg):
        if msg != -1:
            self.imu_msg = msg
        # print(msg)

    def main(self):
        print(f"range_max:{self.laser_msg.range_max}")
        print(f"ori.z :{self.imu_msg.orientation.z}")
        
def main():
    total_data = Total_data()
    while not rospy.is_shutdown():
        total_data.main()

if __name__=="__main__":
    main()




