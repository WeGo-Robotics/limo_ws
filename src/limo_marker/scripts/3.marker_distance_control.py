#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist


class Distance_Control:
    def __init__(self):
        rospy.init_node("alvar_node")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.angle_k = 1
        self.speed = 0
        self.distance = 0.08
        self.rate = rospy.Rate(10)

    def marker_CB(self, data):
        if len(data.markers) != 0:
            ar_pos_z = data.markers[0].pose.pose.position.x
            self.speed = ar_pos_z * self.speed_k
            if abs(ar_pos_z) < self.distance:
                self.speed = 0
        else:
            self.speed = 0

    def main(self):
        print(self.speed)
        self.drive_data.linear.x = self.speed
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        distance_control = Distance_Control()

        while not rospy.is_shutdown():
            alvar_marker_distance_control.main()

    except rospy.ROSInterruptException:
        pass
