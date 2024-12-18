#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node") #1. node 이름 설정
pub = rospy.Publisher("counter",Int32,queue_size=1) #2. node의 역할 설정
rate = rospy.Rate(2) #4. 주기 설정
msg = Int32()
num = 0
while not rospy.is_shutdown():
    num=num+1
    msg.data = num
    pub.publish(msg) #3. publisher - publish
    rate.sleep() # 4-1. 주기대로 실행
    print(msg)