#!/usr/bin/env python3

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
class Class_sub:
    def __init__(self) :
        rospy.init_node("wego_sub_node") #1. node 이름 설정
        rospy.Subscriber("/ar_pose_marker",AlvarMarkers,self.callback) #2. node 역할 설정
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.marker_msg = AlvarMarkers()
        self.cmd_msg = Twist()
        self.marker_id = 0
    def callback(self,msg): #3. subscriber - callback
        self.marker_msg = msg
        if len(self.marker_msg.markers)==0:
            self.cmd_msg.linear.x  = 0
            self.cmd_msg.angular.z = 0
        else :
            x = self.marker_msg.markers[0].pose.pose.position.x
            y = self.marker_msg.markers[0].pose.pose.position.y
            z = self.marker_msg.markers[0].pose.pose.position.z
        
            self.cmd_msg.linear.x = x
            self.cmd_msg.angular.z = y*10
            print(self.cmd_msg.linear.x)
        self.pub.publish(self.cmd_msg)


class_sub = Class_sub()
rospy.spin()