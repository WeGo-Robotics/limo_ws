#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LaserSenser:

    def __init__(self):
        rospy.init_node('laser_example')
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        print(msg.ranges)
        rospy.spin()

if __name__ == '__main__':
    laserSenser = LaserSenser()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass