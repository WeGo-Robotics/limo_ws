#!/usr/bin/env python3

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("ar_pose_marker",AlvarMarkers,self.callback) #2. node 역할 설정
        self.marker_msg = AlvarMarkers()
        self.marker_msg.markers
    def callback(self,msg): #3. subscriber - callback
        self.marker_msg = msg
        try:
            print(self.marker_msg.markers[0].id)
        except:
            pass
        


class_sub = Class_sub()
rospy.spin()