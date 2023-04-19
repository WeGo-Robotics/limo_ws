#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers


# 클래스 생성
class Following_Control:
    # 클래스 초기화 함수 정의
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("alvar_node")
        # /ar_pose_marker 토픽으로부터 AlvarMarkers 메시지를 수신하는 Subscriber 생성
        self.sub_ar = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, callback=self.marker_CB
        )
        # /cmd_vel 토픽에 Twist 메시지 Publisher 생성
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # 1초에 10번 loop 반복
        self.rate = rospy.Rate(10)
        # 변수에 Twist 메시지 할당
        self.drive_data = Twist()
        # angle의 대한 K상수 할당
        self.angle_k = 10
        # speed의 대한 K상수 할당
        self.speed_k = 1
        # 앵글 정의
        self.angle = 0
        # 속도 정의
        self.speed = 0
        # Marker와의 거리
        self.distance = 0.08

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # ar_pos_angle 변수에 position.y(가로) 할당
            ar_pos_angle = data.markers[0].pose.pose.position.y
            # ar_pos_angle * angle_k의 값을 self.angle에 할당하여 값 만큼 횡방향 값 할당
            self.angle = ar_pos_angle * self.angle_k
            # ar_pos_z 변수에 position.x(카메라와 마커의 거리) 할당
            ar_pos_z = data.markers[0].pose.pose.position.x
            # ar_pos_z * speed_k의 값을 self.speed 변수에 할당하여 z값 만큼 종방향 이동
            self.speed = ar_pos_z * self.speed_k
            # ar_pos_z의 값이 self.distance(0.08) 보다 작을경우, 즉 마커의 z좌표가 0.08이하의 경우
            if abs(ar_pos_z) < self.distance:
                # angle을 0으로 설정
                self.angle = 0
                # speed를 0으로 설정
                self.speed = 0
        # 인식하지 않았을 경우 speed, angle을 0으로 설정
        else:
            self.angle = 0
            self.speed = 0

    # 메인 메서드 정의
    def main(self):
        os.system("clear")
        # angle, speed 값을 출력
        print(f"speed : {self.speed}\n")
        print(f"angle : {self.angle}")
        # linear.x 의 값을 self.speed로 변경
        self.drive_data.linear.x = self.speed
        # angular.z의 값을 self.angle로 변경
        self.drive_data.angular.z = self.angle
        # 변경한 drive_data를 /cmd_vel 토픽에 발행
        self.pub.publish(self.drive_data)
        # 1초에 10번 loop 반복
        self.rate.sleep()


if __name__ == "__main__":
    # 클래스 객체 생성
    following_control = Following_Control()
    # 사용자가 작동을 멈추기 전까지 무한 루프
    try:
        while not rospy.is_shutdown():
            # Following_Control 클래스의 main()메서드 호출
            following_control.main()

    except rospy.ROSInterruptException:
        pass
