#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import Twist
from tf.transformations import *
from math import * 

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/scan",LaserScan,self.lidar_cb) #2. node 역할 설정
        rospy.Subscriber("/imu",Imu,self.imu_cb) #2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.lidar_msg = LaserScan()
        self.imu_msg = Imu()
        self.cmd_msg = Twist()
        self.goal_degrees = [-90,0,90,180]
        self.num = 0
        self.goal_degree = 0
        self.state_degree = 0
        self.rate = rospy.Rate(10)

    def lidar_cb(self,msg): #3. subscriber - callback
        self.lidar_msg = msg

    def imu_cb(self,msg): #3. subscriber - callback
       self.imu_msg = msg
       
    def e_stop(self):
        degree_min = self.lidar_msg.angle_min*180/pi
        degree_max = self.lidar_msg.angle_max*180/pi
        degree_increment = self.lidar_msg.angle_increment*180/pi
        degrees = [degree_min+degree_increment*index for index,value in enumerate(self.lidar_msg.ranges)]
        self.obstacle = 0
        for index,value in enumerate(self.lidar_msg.ranges):
            if 0<value<0.5 and abs(degrees[index])<30:
                print(f"장애물:{degrees[index]}")
                self.obstacle=self.obstacle+1

    def cal_angle(self):
        x = self.imu_msg.orientation.x
        y = self.imu_msg.orientation.y
        z = self.imu_msg.orientation.z
        w = self.imu_msg.orientation.w        
        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])
        self.state_degree = yaw*180/pi
        self.goal_degree = self.goal_degrees[self.num]
        print("state_degree:",self.state_degree)
        print(f"goal_degrees:{self.goal_degree}")
    
    def run(self):
        if self.obstacle > 0:
            self.cmd_msg.angular.z = 0
        else :
            self.cmd_msg.angular.z = (self.goal_degree-self.state_degree)*0.05
            if self.goal_degree-0.5 < self.state_degree < self.goal_degree+0.5:
                self.num=(self.num+1)%4
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()
            


class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.e_stop()
    class_sub.cal_angle()
    class_sub.run()