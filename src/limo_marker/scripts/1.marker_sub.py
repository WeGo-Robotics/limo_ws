#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


class Alvar_Marker_Sub:
    def __init__(self):
        rospy.init_node("alvar_node", anonymous=True)

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)

    def marker_CB(self, data):
        print(data.markers)


if __name__ == "__main__":
    alvar_marker_sub = Alvar_Marker_Sub()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
