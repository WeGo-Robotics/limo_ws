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
        avoid_degree = []
        avoid_index=[]
        ctrl_msg = Twist()
        os.system('clear')
        degrees = [(msg.angle_min + msg.angle_increment*index)*180/pi for index, value in enumerate(msg.ranges)]
        # print(degrees)
        for index, value in enumerate(msg.ranges):
            if abs(degrees[index]) <15 and 0 < msg.ranges[index] < 0.15:
                e_stop_check+=1
            elif abs(degrees[index]) <= 90 and  0 < msg.ranges[index] < 0.25:
                avoid_check+=1
                avoid_degree.append(degrees[index])
                avoid_index.append(index)
            else:
                pass

        if e_stop_check>10:
            print("Stop")
            ctrl_msg.linear.x = 0
        # 인덱스의 수 -> 측정된 각도의 갯수 의미 -> 물체의 크기
        elif avoid_check > 15:
            print("Warning")
            ctrl_msg.linear.x = 0.0
            obstacle_start = avoid_index[0]
            obstacle_end = avoid_index[-1]
            right_space = obstacle_start
            left_space = 450-obstacle_end
            
            print(f"right_space:{right_space}")
            print(f"left_space:{left_space}")
            if right_space > left_space:
                print("Go-Right")
                steer_angle = degrees[obstacle_start//2]*0.01
                ctrl_msg.angular.z = -1
            else:
                print("Go-Left")
                ctrl_msg.angular.z = 1
                steer_angle = degrees[(450+obstacle_end)//2]*0.01
            print(steer_angle)
            # print(avoid_index)
        else :
            print("Safe")
            ctrl_msg.linear.x = 1

        self.ctrl_pub.publish(ctrl_msg)

def main():
    e_stop=E_STOP()
    rospy.spin()

if __name__=="__main__":
    main()
