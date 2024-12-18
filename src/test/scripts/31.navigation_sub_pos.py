#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import *
from math import *
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.callback) #2. node 역할 설정
        self.amcl_msg = PoseWithCovarianceStamped()
        # self.amcl_msg.pose.pose.orientation.
    def callback(self,msg): #3. subscriber - callback
        self.amcl_msg = msg
        pos_x = self.amcl_msg.pose.pose.position.x
        pos_y = self.amcl_msg.pose.pose.position.y
        x = self.amcl_msg.pose.pose.orientation.x
        y = self.amcl_msg.pose.pose.orientation.y
        z = self.amcl_msg.pose.pose.orientation.z
        w = self.amcl_msg.pose.pose.orientation.w
        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])
        print(f"x:{pos_x}\ny:{pos_y}\ndegree:{yaw*180/pi}")
        # print(msg)

class_sub = Class_sub()
rospy.spin()