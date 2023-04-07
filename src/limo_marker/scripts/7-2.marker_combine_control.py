#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *


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
        self.rate = rospy.Rate(5)
        self.wait_duration = 3
        self.wait_start = 0
        self.wait_end = 0
        self.wait_ignore = 6
        self.wait_flag = False

    def marker_CB(self, data):
        if len(data.markers) != 0:
            for marker in data.markers:
                if marker.id == 0:
                    print("FAST")
                    self.speed = self.basic_speed * 2
                elif marker.id == 1:
                    print("SLOW")
                    self.speed = self.basic_speed / 2
                elif marker.id == 2:
                    print("STOP")
                    self.speed = 0.0

                elif marker.id == 3:
                    speed, case, time = self.calc_wait()
                    self.speed = speed
                    if case != "CHANGE NORMAL":
                        print(f"{case} : {time:.2f}초")
                    else:
                        print(f"{case}")

                elif marker.id == 4:
                    print("RIGHT")
                    self.angle = -self.basic_angle * pi / 180
                elif marker.id == 5:
                    print("LEFT")
                    self.angle = self.basic_angle * pi / 180

                elif marker.id == 6:
                    print("FOLLOW")
                    ar_pos_z = data.markers[0].pose.pose.position.x
                    ar_pos_angle = data.markers[0].pose.pose.position.y
                    self.speed = ar_pos_z * self.speed_k
                    self.angle = ar_pos_angle * self.angle_k

                    if abs(ar_pos_z) < self.distance:
                        self.angle = 0
                        self.self.speed = 0

            if marker.id != 3:
                self.wait_flag = False
        else:
            self.speed = 0
            self.angle = 0

    def calc_wait(self):
        if self.wait_flag == False:
            self.wait_start = rospy.get_time()
            self.wait_flag = True

        wait_time = self.loop_time - self.wait_start
        ignore_time = self.loop_time - self.wait_end

        if wait_time <= self.wait_duration:
            case = "WAIT"
            speed = 0.0
            self.wait_end = self.loop_time
            time = wait_time

        elif ignore_time < self.wait_ignore:
            case = "IGNORE"
            speed = self.basic_speed * 2
            time = ignore_time
        else:
            case = "CHANGE NORMAL"
            speed = self.speed
            time = 0
            self.wait_flag = False
        return speed, case, time

    def main(self):
        self.loop_time = rospy.get_time()
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
