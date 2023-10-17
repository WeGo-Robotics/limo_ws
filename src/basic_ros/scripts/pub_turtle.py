#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("wego_pub_node")
pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = Twist()
    msg.linear.x = 1
    pub.publish(msg)
    rate.sleep()
