#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist


class STOP_Control:
    def __init__(self):
        rospy.init_node("alvar_node")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.basic_speed = 0.3

    def marker_CB(self, data):
        self.loop_time = rospy.get_time()
        if len(data.markers) != 0:
            if data.markers == 0:
                print("STOP")
                self.drive_data.linear.x = 0.0
        else:
            self.drive_data.linear.x = self.basic_speed

        self.pub.publish(self.drive_data)


if __name__ == "__main__":
    # Create Control class object
    stop_control = STOP_Control()
    try:
        # Continuously call callback function in an infinite loop_time
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
