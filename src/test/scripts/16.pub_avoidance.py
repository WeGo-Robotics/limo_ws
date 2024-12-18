#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import * 

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        rospy.Subscriber("/scan",LaserScan,self.callback) #2. node 역할 설정
        self.lidar_msg = LaserScan()
        self.cmd_msg = Twist()

    def ctrl(self):
        self.cmd_msg.angular.z = 0
        degree_min = self.lidar_msg.angle_min*180/pi
        degree_max = self.lidar_msg.angle_max*180/pi
        degree_increment = self.lidar_msg.angle_increment*180/pi
        degrees = [degree_min+degree_increment*index for index,value in enumerate(self.lidar_msg.ranges)]
        self.obstacle = []
        for index,value in enumerate(self.lidar_msg.ranges):
            if 0<value<0.5 and abs(degrees[index])<60:
                # print(f"장애물:{degrees[index]}")
                self.obstacle.append(index)
            else :
                pass
        
        if self.obstacle != []:
            right_side = self.obstacle[0]
            left_side = len(self.lidar_msg.ranges)-self.obstacle[-1]
            print(f"right_side:{right_side}")
            print(f"left_side:{left_side}")
            if left_side<100 and right_side<100 :
                self.cmd_msg.linear.x = -0.5
                self.cmd_msg.angular.z = -0.5
            else :
                if right_side > left_side:
                    self.cmd_msg.angular.z = -0.5
                else :    
                    self.cmd_msg.angular.z = 0.5
            self.cmd_msg.linear.x = 0
        else :
            self.cmd_msg.linear.x = 0.3
            
        self.pub.publish(self.cmd_msg)

    def callback(self,msg): #3. subscriber - callback
        self.lidar_msg = msg

class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.ctrl()