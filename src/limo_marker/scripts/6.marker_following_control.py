#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers


class Following_Control:
    def __init__(self):
        rospy.init_node("alvar_node")
        self.sub_ar = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, callback=self.marker_CB
        )
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.drive_data = Twist()
        self.angle_k = 10
        self.speed_k = 1
        self.angle = 0
        self.speed = 0
        self.distance = 0.08

    def marker_CB(self, data):
        if len(data.markers) != 0:
            ar_pos_angle = data.markers[0].pose.pose.position.y
            self.angle = ar_pos_angle * self.angle_k
            ar_pos_z = data.markers[0].pose.pose.position.x
            self.speed = ar_pos_z * self.speed_k
            if abs(ar_pos_z) < self.distance:
                self.angle = 0
                self.speed = 0
        else:
            self.angle = 0
            self.speed = 0

    def main(self):
        print(f"angle : {self.angle}\n")
        print(f"speed : {self.speed}")
        self.drive_data.linear.x = self.speed
        self.drive_data.angular.z = self.angle
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    following_control = Following_Control()
    try:
        while not rospy.is_shutdown():
            following_control.main()

    except rospy.ROSInterruptException:
        pass
