#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import *
from math import *
class Class_pub:
    def __init__(self) :
        rospy.init_node("wego_pub_node") #1. node 이름 설정
        rospy.Subscriber("/imu",Imu,self.callback) #2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.imu_msg = Imu()
        self.cmd_msg = Twist()
        self.imu_msg.orientation
        self.goal_degrees = [180,-90,0,90]
        self.num = 0
        self.rate = rospy.Rate(10)
    def run(self):
        x = self.imu_msg.orientation.x
        y = self.imu_msg.orientation.y
        z = self.imu_msg.orientation.z
        w = self.imu_msg.orientation.w        
        roll,pitch,yaw = euler_from_quaternion([x,y,z,w])
        state_degree = yaw*180/pi

        goal_degree = self.goal_degrees[self.num]
        # goal_degree = 45
        if goal_degree-0.5 < state_degree < goal_degree+0.5:
            self.num=(self.num+1)%4
        print(f"goal_degrees:{goal_degree}")
        self.cmd_msg.angular.z = (goal_degree-state_degree)*0.05
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()
        print("yaw",yaw*180/pi)
    def callback(self,msg): #3. subscriber - callback
        self.imu_msg = msg
class_pub = Class_pub()
while not rospy.is_shutdown():
    class_pub.run()