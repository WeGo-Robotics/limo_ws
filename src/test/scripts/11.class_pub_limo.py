import rospy#!/usr/bin/env python3
from geometry_msgs.msg import Twist

class Class_pub:
    def __init__(self):
        rospy.init_node("wego_pub_node") #1. node 이름 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1) #2. node의 역할 설정
        self.rate = rospy.Rate(2) #4. 주기 설정
        self.msg = Twist()

    def run(self):
        while not rospy.is_shutdown():
            self.msg.angular.z = -1
            self.pub.publish(self.msg) #3. publisher - publish
            self.rate.sleep() # 4-1. 주기대로 실행
            print(self.msg)

class_pub = Class_pub()
class_pub.run()