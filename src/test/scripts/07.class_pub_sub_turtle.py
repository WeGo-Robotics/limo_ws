#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Class_pub:
    def __init__(self):
        rospy.init_node("wego_pub_node")  # 1. node 이름 설정
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # 2. node 역할 설정
        self.rate = rospy.Rate(60)  # 4. 주기 설정
        self.cmd_msg = Twist()
        self.pose_msg = Pose()
        self.num = 0

    def run(self):

        print(self.pose_msg.x)
        print(self.pose_msg.y)
        if self.pose_msg.x < 8:
            self.cmd_msg.linear.x = 1
        else:
            self.cmd_msg.linear.x = 0
        self.pub.publish(self.cmd_msg)  # 3. publisher - publish
        self.rate.sleep()  # 4-1. 주기대로 실행
        print(self.cmd_msg)

    def callback(self, msg):  # 3. subscriber - callback
        # print(msg)
        self.pose_msg = msg


class_pub = Class_pub()
while not rospy.is_shutdown():
    class_pub.run()
