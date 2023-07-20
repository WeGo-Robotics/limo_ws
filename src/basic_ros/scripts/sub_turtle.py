#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

rospy.init_node("wego_sub_node")


def callback(msg):
    print(msg)


rospy.Subscriber("/turtle1/pose", Pose, callback)
rospy.spin()
