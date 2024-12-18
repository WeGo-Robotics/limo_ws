#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        self.pub = rospy.Publisher("/joint_states",JointState,queue_size=1)
        self.joint_msg = JointState()
        self.joint_msg.name = ["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
        self.rate = rospy.Rate(0.5)
        self.num = 0
    
    def run(self):
        self.num = 0
        joint1 = self.num
        joint2 = 0.0
        joint3 = 0.0
        joint4 = 0.0
        joint5 = 0.0
        joint6 = 0.0
        joints = [joint1,joint2,joint3,joint4,joint5,joint6]
        self.joint_msg.position = joints
        self.joint_msg.velocity = [0]
        self.joint_msg.effort = [0]
        self.pub.publish(self.joint_msg)
        self.rate.sleep()
        print(self.joint_msg)

class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.run()