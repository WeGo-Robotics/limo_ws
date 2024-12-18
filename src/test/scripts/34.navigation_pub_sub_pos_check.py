#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from actionlib_msgs.msg import GoalStatusArray
from math import *
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        self.pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1) #2. node 역할 설정
        rospy.Subscriber("move_base/status",GoalStatusArray,self.goal_status_cb)
        self.goal_msg = PoseStamped()
        self.goal_status_msg = GoalStatusArray()
        self.goal_status_msg.status_list
        self.goal_msg.header.frame_id = "map"
        self.rate = rospy.Rate(1)
        self.goal_pos_x = -3
        self.msg_flag = False
        self.goal_flag = False
        self.goal_status = 0
    def run(self):
        if self.msg_flag == True:
            if self.goal_status == 3 and self.goal_flag == False:
                self.goal_pos_x -= 0.5
                self.goal_pos_y = -1.0
                print(self.goal_status)
                # self.goal_pos_x = 
                self.goal_msg.pose.position.x = self.goal_pos_x
                self.goal_msg.pose.position.y = self.goal_pos_y

                yaw_degree = 60
                yaw = yaw_degree/180*pi

                x,y,z,w = quaternion_from_euler(0,0,yaw)

                self.goal_msg.pose.orientation.x = x
                self.goal_msg.pose.orientation.y = y
                self.goal_msg.pose.orientation.z = z
                self.goal_msg.pose.orientation.w = w
                print(self.goal_msg)

                self.pub.publish(self.goal_msg)
                self.rate.sleep()
                self.goal_flag = True

    def goal_status_cb(self,msg):
        if msg!=None:
            self.msg_flag = True
            self.goal_status_msg = msg
            if len(self.goal_status_msg.status_list) == 0:
               self.goal_status = 3

            else:
                self.goal_status = self.goal_status_msg.status_list[0].status
                if self.goal_status == 1:
                    self.goal_flag = False


class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.run()