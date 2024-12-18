#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import *
from math import *
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("imu",Imu,self.callback) #2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.imu_msg = Imu()
        self.cmd_msg = Twist()
        self.imu_msg.orientation
    
    def callback(self,msg): #3. subscriber - callback
        self.cmd_msg.angular.z = 0
        print(msg)
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])
        print("yaw",yaw*180/pi)
        
        state_degree = yaw*180/pi
        goal_degree = 90
        self.cmd_msg.angular.z = (goal_degree-state_degree)*0.1
        self.pub.publish(self.cmd_msg)
        
class_sub = Class_sub()
rospy.spin()