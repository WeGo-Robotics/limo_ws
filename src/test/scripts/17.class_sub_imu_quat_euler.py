#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *
from math import *
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("imu",Imu,self.callback) #2. node 역할 설정
        self.imu_msg = Imu()
        self.imu_msg.orientation
    def callback(self,msg): #3. subscriber - callback
        print(msg)
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])
        print("yaw",yaw*180/pi)

class_sub = Class_sub()
rospy.spin()