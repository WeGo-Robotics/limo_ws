#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node")  # ROS 1단계(필수) : 노드 이름
pub = rospy.Publisher("/counter", Int32, queue_size=1)  # ROS 2단계(필수) : 노드 역할 - 퍼블리셔 설정
rate = rospy.Rate(10)  # ROS 2-1단계(옵션) : 퍼블리셔 - 주기 설정
msg = Int32()  # 메세지 타입 설정 및 초기화

while not rospy.is_shutdown():
    pub.publish(msg)  # ROS 3단계(필수) : 퍼블리셔 - 퍼블리시 실행
    msg.data += 1  # 메세지 항목(데이터) - 데이터 값 변경
    print(f"{msg}")  # 출력 : 메세지
    print(f"{msg.data}")  # 출력 : 메세지 항목(데이터)
    rate.sleep()  # ROS 3-1단계(옵션) : 퍼블리셔 - 주기 실행
