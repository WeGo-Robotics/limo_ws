#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import *


# 클래스 생성
class ID_control:
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
        # 속도 정의
        self.speed = 0
        # 앵글 정의
        self.angle = 0
        # angle의 대한 K상수 할당
        self.angle_k = 10
        # speed의 대한 K상수 할당
        self.speed_k = 1
        # 1초에 10번 loop 반복
        self.rate = rospy.Rate(10)
        # flag 변수 선언
        self.wait_flag = False

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        control_angle = 0
        control_speed = 0
        # loop_time에 현재 ROS 시간 저장
        self.loop_time = rospy.get_time()
        # data.markers 문자열의 길이가 0이 아닐 경우 조건문 실행
        if len(data.markers) != 0:
            os.system("clear")
            # data.markers 에 있는 마커 정보를 처리
            for marker in data.markers:
                # id가 0번일 경우
                if marker.id == 0:
                    # "FAST"출력
                    print("FAST")
                    # 기준 속도 * 2
                    control_speed = self.basic_speed * 2
                # id가 1번일 경우
                elif marker.id == 1:
                    # "SLOW" 출력
                    print("SLOW")
                    # 기준 속도 / 2
                    control_speed = self.basic_speed / 2
                # id가 2번일 경우
                elif marker.id == 2:
                    # "STOP" 출력
                    print("STOP")
                    # 속도 0으로 감속
                    control_speed = 0.0
                # id가 3일 경우
                elif marker.id == 3:
                    # flag 함수가 False 경우
                    if self.wait_flag == False:
                        # True로 변경
                        self.wait_flag = True
                        # ROS 시간 저장
                        self.wait_time = rospy.get_time()
                    # flag 함수가 False가 아닐 경우
                    else:
                        # 현재 시간과 기다린 시간의 차이가 3초 미만
                        if self.loop_time - self.wait_time < 3:
                            # 속도 0으로 감속
                            control_speed = 0
                            # "WAIT" 출력, 기다린 시간 출력
                            print("WAIT", self.loop_time - self.wait_time)
                            # stop_time에 현재 ROS 시간 저장
                            self.stop_time = rospy.get_time()
                        # 3초가 넘었을 경우
                        else:
                            control_speed = self.basic_speed
                            # 현재 시간과 멈춘 시간이 5초 초과
                            if self.loop_time - self.stop_time > 5:
                                # wait_flag 함수를 False 다시 변경하여 3번 마커 인식했을 때 조건문 실행
                                self.wait_flag = False
                # id가 4번일 경우
                elif marker.id == 4:
                    # "RIGHT" 출력
                    print("RIGHT")
                    # 바퀴를 오른쪽으로 -0.2618 라디안 만큼 회전 (-15도)
                    control_angle = -self.basic_angle * pi / 180
                # id가 5번일 경우
                elif marker.id == 5:
                    # "LEFT" 출력
                    print("LEFT")
                    # 바퀴를 왼쪽으로 0.2618 라디안 만큼 회전 (15도)
                    control_angle = self.basic_angle * pi / 180
        # 인식 되지 않았을 경우
        else:
            control_angle = 0
            control_speed = 0

        self.speed = control_speed
        self.angle = control_angle

    # 메인 메서드 정의
    def main(self):
        # linear.x 의 값을 self.speed로 변경
        self.drive_data.linear.x = self.speed
        # angular.z의 값을 self.angle로 변경
        self.drive_data.angular.z = self.angle
        # 변경한 drive_data를 /cmd_vel 토픽에 발행
        self.pub.publish(self.drive_data)
        # 1초에 10번 loop 반복
        self.rate.sleep()


if __name__ == "__main__":
    try:
        # 클래스 객체 생성
        id_control = ID_control()
        # 사용자가 작동을 멈추기 전까지 무한 루프
        while not rospy.is_shutdown():
            # ID_control 클래스의 main()메서드 호출
            id_control.main()

    # 예외가 발생하면, 예외를 무시하고 프로그램 종료
    except rospy.ROSInterruptException:
        pass
