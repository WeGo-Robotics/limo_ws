#!/usr/bin/env python3 

import rospy  # module for using Python in ROS
from geometry_msgs.msg import Twist  # import the Twist message type from the geometry_msgs package in ROS
from sensor_msgs.msg import LaserScan  # import the LaserScan message type from the sensor_msgs package in ROS
from math import *  # module for using math functions
from time import *  # module for using time functions

class Limo_estop:  # class declaration
    def __init__(self):  # constructor method
        rospy.init_node("laser_scan_node")  # initialize ROS node, named "laser_scan_node"
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)  # subscribe to /scan topic in ROS, and call laser_callback method every time new data is received
        self.rate=rospy.Rate(30)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)  # create a publisher object for publishing to /cmd_vel topic in ROS, with message type Twist and queue size of 3
        self.lidar_flag = False  # variable that stores whether the lidar data has come in for the first time, initialized to False
        self.deg = 10  # variable that stores how many degrees to check to the left and right of the center of the lidar
        self.cmd_vel_msg = Twist()  # create a Twist message object
        

    def laser_callback(self, msg):
        num = 0 # initialize num variable
        # when the lidar data comes in for the first time, calculate the degrees and store them in degrees list
        if self.lidar_flag == False:
            self.degrees = [
                (msg.angle_min + (i * msg.angle_increment)) * 180 / pi
                for i, data in enumerate(msg.ranges)
            ]
            self.lidar_flag = True
        # calculate the number of infrared sensors using the ranges and degrees list of lidar data
        for i, data in enumerate(msg.ranges):
            if -self.deg < self.degrees[i] < self.deg and 0 < msg.ranges[i] < 0.5:
                num += 1
        # if the number of infrared sensors is less than 10, set the movement speed to 0.5
        if num < 10:
            self.cmd_vel_msg.linear.x = 0.15
            
            print({})
        # otherwise, set the movement speed to 0 and print STOP message
        else:
            self.cmd_vel_msg.linear.x = 0
        self.pub.publish(self.cmd_vel_msg)
        self.rate.sleep()
if __name__ == "__main__":
    limo_estop = Limo_estop()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
