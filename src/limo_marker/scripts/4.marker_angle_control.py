#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist


class Angle_Control:
    def __init__(self):
        rospy.init_node("alvar_node")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.angle = 0
        self.angle_k = 10
        self.rate = rospy.Rate(10)

    def marker_CB(self, data):
        if len(data.markers) != 0:
            ar_pos_x = data.markers[0].pose.pose.position.y
            self.angle = ar_pos_x * self.angle_k
        else:
            self.angle = 0

    def main(self):
        print(self.angle)
        self.drive_data.angular.z = self.angle
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        angle_control = Angle_Control()

        while not rospy.is_shutdown():
            angle_control.main()

    except rospy.ROSInterruptException:
        pass
