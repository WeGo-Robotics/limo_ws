#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
import os

# 클래스 생성
class Lidar: 

# 초기화 및 초기 설정 
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        rospy.Subscriber("/scan",LaserScan,self.lidar_CB)
        lidar_msg = LaserScan()
        self.ctrl_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=3)
        self.ctrl_msg = Twist()
    # 함수 설정
    def lidar_CB(self,msg):
        # print(msg)
        degrees = [(msg.angle_min + msg.angle_increment*index)*180/pi for index, value in enumerate(msg.ranges)]
        # print(degrees)
        e_stop_check=0
        for index, value in enumerate(msg.ranges):
            if abs(degrees[index]) <30 and 0 < msg.ranges[index] < 0.60:
                e_stop_check+=1
            else:
                pass
        print(e_stop_check)
        if e_stop_check > 0:
            speed = 0
            print("STOP")
        else :
            speed = 1
            print("GO")
        self.ctrl_msg.linear.x = speed
        self.ctrl_pub.publish(self.ctrl_msg)

def main():
    lidar=Lidar()
    rospy.spin()

if __name__=="__main__":
    main()
