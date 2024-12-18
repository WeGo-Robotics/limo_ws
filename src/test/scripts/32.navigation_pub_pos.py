#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from math import *
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        self.pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1) #2. node 역할 설정
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = 'map'
        self.rate = rospy.Rate(0.5)
        self.pos_x = 0
    def run(self): 
        self.pos_x -= 0.2
        self.pos_y = 0.3
        self.goal_msg.pose.position.x = self.pos_x
        self.goal_msg.pose.position.y = self.pos_y
        
        yaw_degree = 180
        yaw = yaw_degree/180*pi

        x,y,z,w = quaternion_from_euler(0,0,yaw)

        self.goal_msg.pose.orientation.x = x
        self.goal_msg.pose.orientation.y = y
        self.goal_msg.pose.orientation.z = z
        self.goal_msg.pose.orientation.w = w
        print(self.goal_msg)
        self.pub.publish(self.goal_msg)
        self.rate.sleep()

class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.run()