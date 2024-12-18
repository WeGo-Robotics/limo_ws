#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_sub_node") #1. node 이름 설정

def callback(msg): #3. subscriber - callback
    print(msg)

rospy.Subscriber("counter",Int32,callback) #2. node 역할 설정
rospy.spin()