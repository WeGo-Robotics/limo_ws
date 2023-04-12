#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *
import os


# 클래스 생성
class Combine_Control:
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
        # 기준 앵글 각도 정의
        self.basic_angle = 15
        # speed의 대한 K상수 할당
        self.speed_k = 1
        # angle의 대한 K상수 할당
        self.angle_k = 10
        # 속도 정의
        self.speed = 0
        # 앵글 정의
        self.angle = 0
        # Marker와의 거리
        self.distance = 0.08
        # 1초에 10번 loop 반복
        self.rate = rospy.Rate(10)
        # case 정의
        self.case = " "
        # 이전 case 정의
        self.previous_case = " "
        # 이전 시간 정의
        self.previous_time = 0
        # wait 시간
        self.wait_duration = 3
        # wait 후 지나갈 시간
        self.wait_ignore = 5

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # data.markers 에 있는 마커 정보를 처리
            for marker in data.markers:
                # id 3 미만일 경우
                if marker.id <= 3:
                    self.angle = 0
                # id 4번 또는 5번일 경우
                elif marker.id == 4 or 5:
                    self.speed = 0
                # 그 외 통과
                else:
                    pass

                # id 0번일 경우
                if marker.id == 0:
                    # "FAST" 출력
                    self.case = "FAST"
                    # 기준 속도 * 2
                    self.speed = self.basic_speed * 2
                # id가 1번일 경우
                elif marker.id == 1:
                    # "SLOW" 출력
                    self.case = "SLOW"
                    # 기준 속도 / 2
                    self.speed = self.basic_speed / 2
                # id가 2번일 경우
                elif marker.id == 2:
                    # "STOP" 출력
                    self.case = "STOP"
                    # 속도 0으로 감속
                    self.speed = 0.0
                # id가 3번일 경우
                elif marker.id == 3:
                    # time_gap이 duration(3)보다 이하
                    if self.time_gap <= self.wait_duration:
                        self.case = "WAIT"
                        self.speed = 0.0
                    # time_gap이 ignore(5) + duration(3) 미만
                    elif self.time_gap < self.wait_ignore + self.wait_duration:
                        self.case = "IGNORE"
                        self.speed = self.basic_speed * 2
                    # 3번 마커가 사라졌을 경우 NOMAL speed로 변경
                    else:
                        self.case = "CHANGE NORMAL"
                        self.speed = self.speed
                # id가 4번일 경우
                elif marker.id == 4:
                    self.case = "RIGHT"
                    # 바퀴를 오른쪽으로 -0.2618 라디안 만큼 회전 (-15도)
                    self.angle = -self.basic_angle * pi / 180
                elif marker.id == 5:
                    self.case = "LEFT"
                    # 바퀴를 왼쪽으로 0.2618 라디안 만큼 회전 (15도)
                    self.angle = self.basic_angle * pi / 180
                # id가 6번일 경우
                elif marker.id == 6:
                    self.case = "FOLLOW"
                    # ar_pos_z 변수에 position.x(카메라와 마커의 거리) 할당
                    ar_pos_z = data.markers[0].pose.pose.position.x
                    # ar_pos_angle 변수에 position.y(가로) 할당
                    ar_pos_angle = data.markers[0].pose.pose.position.y
                    # ar_pos_z * speed_k의 값을 self.speed 변수에 할당하여 z값 만큼 종방향 이동
                    self.speed = ar_pos_z * self.speed_k
                    # ar_pos_angle * angle_k의 값을 self.angle에 할당하여 값 만큼 횡방향 값 할당
                    self.angle = ar_pos_angle * self.angle_k
                    # ar_pos_z의 값이 self.distance(0.08) 보다 작을경우, 즉 마커의 z좌표가 0.08이하의 경우
                    if abs(ar_pos_z) < self.distance:
                        self.angle = 0
                        self.self.speed = 0
        # 인식하지 않았을 경우 case는 MOMAL, speed, angle을 0으로 설정
        else:
            self.case = "NORMAL"
            self.speed = 0
            self.angle = 0

    # 클래스의 메서드 정의
    def compare_case(self):
        # 이전 case가 현재 case와 같지 않을 경우 조건문 실행
        if self.previous_case != self.case:
            # 이전 case가 WAIT, 그리고 현재 case가 IGNORE일 경우
            if self.previous_case == "WAIT" and self.case == "IGNORE":
                # 실행중인 조건문 통과하여 마커가 3번일 경우 조건문 계속 실행
                pass
            # 그 외
            else:
                # 이전 case 업데이트
                self.previous_case = self.case
                # 이전 시간 업데이트
                self.previous_time = rospy.get_time()
        # time_gap 에 현재 ROS시간 - 이전 ROS시간
        self.time_gap = self.loop_time - self.previous_time

    # 클래스의 메서드 정의, 개체의 상태를 터미널에 인쇄합니다.
    def print_state(self):
        # 터미널 초기화
        os.system("clear")
        print(f"--------------")
        print(f"case : {self.case}")
        # case가 IGNORE일 경우
        if self.case == "IGNORE":
            # 무시하고 지나가는 시간 출력
            print(f"time : {self.time_gap-self.wait_duration:.2f}")
        else:
            print(f"time : {self.time_gap:.2f}")
        print(f"--------------")

    # 메인 메서드 정의
    def main(self):
        # 현재 ROS 시간 할당
        self.loop_time = rospy.get_time()
        self.compare_case()
        self.print_state()
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
    combine_control = Combine_Control()
    # 사용자가 작동을 멈추기 전까지 무한 루프
    try:
        while not rospy.is_shutdown():
            # Combine_Control 클래스의 main()메서드 호출
            combine_control.main()

    except rospy.ROSInterruptException:
        pass
