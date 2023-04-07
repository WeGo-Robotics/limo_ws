#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *


class ID_control:
    def __init__(self):
        rospy.init_node("alvar_node")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.basic_speed = 0.3
        self.basic_angle = 15
        self.speed = 0
        self.angle = 0
        self.angle_k = 10
        self.speed_k = 1
        self.rate = rospy.Rate(10)
        self.wait_flag = False

    def marker_CB(self, data):
        self.loop_time = rospy.get_time()
        if len(data.markers) != 0:
            for marker in data.markers:
                if marker.id == 0:
                    print("FAST")
                    control_speed = self.basic_speed * 2
                elif marker.id == 1:
                    print("SLOW")
                    control_speed = self.basic_speed / 2
                elif marker.id == 2:
                    print("STOP")
                    control_speed = 0.0
                elif marker.id == 3:
                    if self.wait_flag == False:
                        self.wait_flag = True
                        self.wait_time = rospy.get_time()
                    else:
                        if self.loop_time - self.wait_time < 3:
                            control_speed = 0
                            print("WAIT", self.loop_time - self.wait_time)
                            self.stop_time = rospy.get_time()
                        else:
                            control_speed = self.basic_speed

                            if self.loop_time - self.stop_time > 5:
                                self.wait_flag = False
                elif marker.id == 4:
                    print("RIGHT")
                    control_angle = -self.basic_angle * pi / 180

                elif marker.id == 5:
                    print("LEFT")
                    control_angle = self.basic_angle * pi / 180
        else:
            control_angle = 0
            control_speed = 0

        self.speed = control_speed
        self.angle = control_angle

    def main(self):
        self.drive_data.linear.x = self.speed
        self.drive_data.angular.z = self.angle
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        id_control = ID_control()

        while not rospy.is_shutdown():
            id_control.main()

    except rospy.ROSInterruptException:
        pass
