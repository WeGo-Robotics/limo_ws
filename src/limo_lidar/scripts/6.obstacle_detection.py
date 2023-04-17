#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *


class Detect_Objects:
    def __init__(self):
        rospy.init_node("laser_scan_node")
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.msg = LaserScan()
        self.lidar_flag = False
        self.rate = rospy.Rate(10)
        self.last_degree = 0
        self.filter_degree = 1
        self.standard_degree = 10
        self.FOV_degree = 60
        self.FOV_range = 0.5
        self.nuber_of_object = 0

    def laser_callback(self, msg):
        if self.lidar_flag == False:
            self.degrees = [
                (msg.angle_min + index * msg.angle_increment) * 180 / pi
                for index, value in enumerate(msg.ranges)
            ]
            self.lidar_flag = True
        self.msg = msg

    def lidar_filter(self, msg):
        detect_degree = []
        for index, value in enumerate(msg.ranges):
            if (
                -self.FOV_degree < self.degrees[index] < self.FOV_degree
                and 0 < msg.ranges[index] < self.FOV_range
            ):
                current_range = msg.ranges[index]
                current_degree = self.degrees[index]
                if abs(current_degree - self.last_degree) > self.filter_degree:
                    self.last_degree = current_degree
                    detect_degree.append(current_degree)
        return detect_degree

    def count_object_function(self, detect_degree):
        count_objects = 0
        if len(detect_degree) != 0:
            count_objects = 1
            for index in range(len(detect_degree) - 1):
                between_degree = abs(detect_degree[index] - detect_degree[index + 1])
                if self.standard_degree < between_degree:
                    count_objects += 1
        return count_objects

    def print_object_num(self, count_objects):
        if self.nuber_of_object != count_objects:
            print("------------------------------")
            print(f"past objects : {self.nuber_of_object}")
            if self.nuber_of_object < count_objects:
                print(f"new object : {count_objects-self.nuber_of_object}")
            else:
                print(f"disappear object : {self.nuber_of_object - count_objects}")
            self.nuber_of_object = count_objects
            print(f"current objects : {self.nuber_of_object}")
            print("------------------------------")

    def main(self):
        detect_degree = self.lidar_filter(self.msg)
        count_objects = self.count_object_function(detect_degree)
        self.print_object_num(count_objects)


if __name__ == "__main__":
    detect_objects = Detect_Objects()
    try:
        while not rospy.is_shutdown():
            detect_objects.main()

    except rospy.ROSInterruptException:
        pass