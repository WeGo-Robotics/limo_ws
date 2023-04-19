#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *


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
        self.speed = 0
        self.angle = 0
        # Marker와의 거리
        self.distance = 0.08
        self.rate = rospy.Rate(5)
        self.wait_duration = 3
        self.wait_start = 0
        self.wait_end = 0
        self.wait_ignore = 6
        self.wait_flag = False

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            # data.markers 에 있는 마커 정보를 처리
            for marker in data.markers:
                # id가 0번일 경우
                if marker.id == 0:
                    print("FAST")
                    self.speed = self.basic_speed * 2
                # id가 1번일 경우
                elif marker.id == 1:
                    print("SLOW")
                    self.speed = self.basic_speed / 2
                # id가 2번일 경우
                elif marker.id == 2:
                    print("STOP")
                    self.speed = 0.0
                # id가 3번일 경우
                elif marker.id == 3:
                    # speed, case, time에 calc_wait 메서드 할당
                    speed, case, time = self.calc_wait()
                    self.speed = speed
                    # case가 "CHANGE NORMAL"과 다를 경우
                    if case != "CHANGE NORMAL":
                        print(f"{case} : {time:.2f}초")
                    else:
                        print(f"{case}")
                # id가 4번일 경우
                elif marker.id == 4:
                    print("RIGHT")
                    # 바퀴를 오른쪽으로 -0.2618 라디안 만큼 회전 (-15도)
                    self.angle = -self.basic_angle * pi / 180
                # id가 5번일 경우
                elif marker.id == 5:
                    print("LEFT")
                    # 바퀴를 왼쪽으로 0.2618 라디안 만큼 회전 (15도)
                    self.angle = self.basic_angle * pi / 180
                # id가 6번일 경우
                elif marker.id == 6:
                    print("FOLLOW")
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

            # id가 3번과 다를 경우
            if marker.id != 3:
                self.wait_flag = False
        else:
            self.speed = 0
            self.angle = 0

    # 클래스의 메서드 정의
    def calc_wait(self):
        # id 3번 마커가 인식되고, wait_flag가 False일 경우
        if self.wait_flag == False:
            # flag를 True로 변경
            self.wait_flag = True
            # ROS시간 저장
            self.wait_start = rospy.get_time()

        # ROS 시간 - 기준 시간
        wait_time = self.loop_time - self.wait_start
        ignore_time = self.loop_time - self.wait_end
        # wait_time이 duration(3) 이하일 경우
        if wait_time <= self.wait_duration:
            case = "WAIT"
            speed = 0.0
            self.wait_end = self.loop_time
            time = wait_time
        # ignore_time이 ignore(6) 미만일 경우
        elif ignore_time < self.wait_ignore:
            case = "IGNORE"
            speed = self.basic_speed * 2
            time = ignore_time
        else:
            case = "CHANGE NORMAL"
            speed = self.speed
            time = 0
            self.wait_flag = False
        return speed, case, time

    # 메인 메서드 정의
    def main(self):
        # 현재 ROS 시간 할당
        self.loop_time = rospy.get_time()
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
