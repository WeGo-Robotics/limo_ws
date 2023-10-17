#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node")
pub = rospy.Publisher("/counter", Int32, queue_size=1)
num = 0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    num += 1
    print(num)
    pub.publish(num)
    rate.sleep()
