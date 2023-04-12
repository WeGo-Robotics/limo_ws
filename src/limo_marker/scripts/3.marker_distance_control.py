#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist


# 클래스 생성
class Distance_Control:
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
        # speed의 대한 K상수 할당
        self.speed_k = 1
        # angle의 대한 K상수 할당
        self.angle_k = 1
        # 속도 정의
        self.speed = 0
        # Marker와의 거리
        self.distance = 0.08
        # 1초에 10번 loop 반복
        self.rate = rospy.Rate(10)

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # ar_pos_z 변수에 position.x(카메라와 마커의 거리) 할당
            ar_pos_z = data.markers[0].pose.pose.position.x
            # ar_pos_z * speed_k의 값을 self.speed 변수에 할당하여 z값 만큼 종방향 값 할당
            self.speed = ar_pos_z * self.speed_k
            # ar_pos_z의 값이 self.distance(0.08) 보다 작을경우, 즉 마커의 z좌표가 0.08인 경우
            if abs(ar_pos_z) < self.distance:
                # speed를 0으로 설정
                self.speed = 0
        # 인식하지 않았을 경우 speed를 0으로 설정
        else:
            self.speed = 0

    # 메인 메서드 정의
    def main(self):
        os.system("clear")
        # 현재 속도 프린트
        print(f"speed : {self.speed}")
        # linear.x의 값을 self.speed의 값으로 변경
        self.drive_data.linear.x = self.speed
        # drive_data를 /cmd_vel 토픽에 발행
        self.pub.publish(self.drive_data)
        # 1초에 10번 loop 반복
        self.rate.sleep()


if __name__ == "__main__":
    try:
        # 클래스 객체 생성
        distance_control = Distance_Control()
        # 사용자가 작동을 멈추기 전까지 무한 루프
        while not rospy.is_shutdown():
            # Distance_Control 클래스의 main()메서드 호출
            distance_control.main()

    # 예외가 발생하면, 예외를 무시하고 프로그램 종료
    except rospy.ROSInterruptException:
        pass
