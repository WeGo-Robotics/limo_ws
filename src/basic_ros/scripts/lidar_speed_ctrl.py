#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
import os

# 클래스 생성
class E_STOP: 

# 초기화 및 초기 설정 
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        self.ctrl_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=3)
        rospy.Subscriber("/scan",LaserScan,self.lidar_CB)
        lidar_msg = LaserScan()
        
    # 함수 설정
    def lidar_CB(self,msg):
        e_stop_check = 0
        avoid_check = 0

        ctrl_msg = Twist()
        os.system('clear')
        degrees = [(msg.angle_min + msg.angle_increment*index)*180/pi for index, value in enumerate(msg.ranges)]
        # print(degrees)
        for index, value in enumerate(msg.ranges):
            if abs(degrees[index]) <15 and 0 < msg.ranges[index] < 0.15:
                e_stop_check+=1
            elif abs(degrees[index]) < 45 and  0 < msg.ranges[index] < 0.30:
                avoid_check +=1
            else:
                pass

        if e_stop_check>10:
            print("Stop")
            ctrl_msg.linear.x = 0

        elif avoid_check > 30:
            print("Warning")
            ctrl_msg.linear.x = 0.5
        else :
            print("Safe")
            ctrl_msg.linear.x = 1

        self.ctrl_pub.publish(ctrl_msg)

def main():
    e_stop=E_STOP()
    rospy.spin()

if __name__=="__main__":
    main()
