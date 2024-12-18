#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Class_sub:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node 이름 설정
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # 2. node 역할 설정
        self.pose_msg = Pose()
        self.pose_msg.linear_velocity
        self.cmd_msg = Twist()

    def callback(self, msg):  # 3. subscriber - callback
        print(msg.x)
        print(msg.y)
        if msg.x < 8:
            self.cmd_msg.linear.x = 1
        else:
            self.cmd_msg.linear.x = 0
        self.pub.publish(self.cmd_msg)


class_sub = Class_sub()
rospy.spin()
