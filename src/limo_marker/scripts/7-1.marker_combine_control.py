#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *
import os


class Combine_Control:
    def __init__(self):
        rospy.init_node("alvar_node")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.basic_speed = 0.3
        self.basic_angle = 15
        self.speed_k = 1
        self.angle_k = 10
        self.speed = 0
        self.angle = 0
        self.distance = 0.08
        self.rate = rospy.Rate(10)
        self.case = " "
        self.previous_case = " "
        self.previous_time = 0
        self.wait_duration = 3
        self.wait_ignore = 5

    def marker_CB(self, data):
        if len(data.markers) != 0:
            for marker in data.markers:
                if marker.id <= 3:
                    self.angle = 0
                elif marker.id == 4 or 5:
                    self.speed = 0
                else:
                    pass

                if marker.id == 0:
                    self.case = "FAST"
                    self.speed = self.basic_speed * 2
                elif marker.id == 1:
                    self.case = "SLOW"
                    self.speed = self.basic_speed / 2
                elif marker.id == 2:
                    self.case = "STOP"
                    self.speed = 0.0
                elif marker.id == 3:
                    if self.time_gap <= self.wait_duration:
                        self.case = "WAIT"
                        self.speed = 0.0
                    elif self.time_gap < self.wait_ignore + self.wait_duration:
                        self.case = "IGNORE"
                        self.speed = self.basic_speed * 2
                    else:
                        self.case = "CHANGE NORMAL"
                        self.speed = self.speed

                elif marker.id == 4:
                    self.case = "RIGHT"
                    self.angle = -self.basic_angle * pi / 180
                elif marker.id == 5:
                    self.case = "LEFT"
                    self.angle = self.basic_angle * pi / 180

                elif marker.id == 6:
                    self.case = "FOLLOW"
                    ar_pos_z = data.markers[0].pose.pose.position.x
                    ar_pos_angle = data.markers[0].pose.pose.position.y
                    self.speed = ar_pos_z * self.speed_k
                    self.angle = ar_pos_angle * self.angle_k

                    if abs(ar_pos_z) < self.distance:
                        self.angle = 0
                        self.self.speed = 0

        else:
            self.case = "NORMAL"
            self.speed = 0
            self.angle = 0

    def compare_case(self):
        if self.previous_case != self.case:
            if self.previous_case == "WAIT" and self.case == "IGNORE":
                pass
            else:
                self.previous_case = self.case
                self.previous_time = rospy.get_time()

        self.time_gap = self.loop_time - self.previous_time

    def print_state(self):
        os.system("clear")
        print(f"--------------")
        print(f"case : {self.case}")
        if self.case == "IGNORE":
            print(f"time : {self.time_gap-self.wait_duration:.2f}")
        else:
            print(f"time : {self.time_gap:.2f}")
        print(f"--------------")

    def main(self):
        self.loop_time = rospy.get_time()
        self.compare_case()
        self.print_state()
        self.drive_data.linear.x = self.speed
        self.drive_data.angular.z = self.angle
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    combine_control = Combine_Control()
    try:
        while not rospy.is_shutdown():
            combine_control.main()

    except rospy.ROSInterruptException:
        pass
