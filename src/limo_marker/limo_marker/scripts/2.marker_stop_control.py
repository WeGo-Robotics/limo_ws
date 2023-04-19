#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist


# 클래스 생성
class STOP_Control:
    # 클래스 초기화 함수 정의
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("alvar_node")
        # /ar_pose_marker 토픽으로부터 AlvarMarkers 메시지를 수신하는 Subscriber 생성
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        # /cmd_vel 토픽에 Twist 메시지 Publisher 생성
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # 변수에 Twist 메시지 할당
        self.drive_data = Twist()
        # 기준 속도 정의
        self.basic_speed = 0.3

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # data.markers 에 있는 마커 정보를 처리
            for marker in data.markers:
                # 마커의 id가 0번일 경우
                if marker.id == 0:
                    os.system("clear")
                    # STOP출력
                    print("STOP")
                    # linear.x 0로 설정
                    self.drive_data.linear.x = 0.0
        # 0번이 아닐 경우 basic_speed로 설정
        else:
            self.drive_data.linear.x = self.basic_speed
        # drive_data를 /cmd_vel 토픽에 발행
        self.pub.publish(self.drive_data)


if __name__ == "__main__":
    # 클래스 객체 생성
    stop_control = STOP_Control()
    try:
        # 함수를 호출하여 ROS 노드를 실행하고, 메시지를 수신하는 동안 무한 루프
        rospy.spin()
    # 예외가 발생하면, 예외를 무시하고 프로그램 종료
    except rospy.ROSInterruptException:
        pass
