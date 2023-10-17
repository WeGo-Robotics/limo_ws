#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 IMU(Inertial Measurement Unit) 데이터를 활용하여 로봇을 제어하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 수학 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from math import *


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/imu" 토픽에서 Imu 메시지를 구독하고, 콜백 함수(imu_CB)를 호출합니다.
        rospy.Subscriber("/imu", Imu, self.imu_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/cmd_vel" 토픽에 Twist 메시지를 발행합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # ROS 2단계: 노드 역할 - 퍼블리셔 설정

        # 메세지 타입 설정 및 초기화
        self.imu_msg = Imu()  # IMU 데이터를 저장하기 위한 메시지
        self.ctrl_msg = Twist()  # 로봇 제어 명령을 저장하기 위한 메시지

        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정
        self.imu_flag = False  # IMU 데이터 수신 확인을 위한 변수 설정
        self.imu_linear_z_ref = 10  # IMU 데이터의 초기 값 설정
        self.imu_linear_z_down = 10  # IMU 데이터의 초기 값 설정
        self.imu_linear_z_up = 10  # IMU 데이터의 초기 값 설정
        self.safe_num = 0  # 로봇이 안전한 상태인지 확인하기 위한 변수

    def imu_CB(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        if msg != -1:  # 메세지가 들어왔을 때:
            self.imu_msg = msg  # 메세지를 self.imu_msg에 저장
            self.imu_flag = True  # IMU 데이터 수신 확인 변수를 True로 설정
        else:  # 메세지가 들어오지 않았을 때:
            self.imu_flag = False  # IMU 데이터 수신 확인 변수를 False로 설정

    def sense(self):  # 인지 함수
        imu_linear_z = self.imu_msg.linear_acceleration.z  # IMU 데이터에서 linear_acceleration.z 값을 변수에 저장
        return imu_linear_z  # 저장한 값을 반환

    def think(self, imu_linear_z):  # 판단 함수
        if imu_linear_z > self.imu_linear_z_up:  # IMU 데이터의 z 선형 가속도가 설정한 상한값보다 클 때:
            self.imu_linear_z_up = imu_linear_z  # 상한값을 현재 값으로 업데이트
        elif imu_linear_z < self.imu_linear_z_down:  # IMU 데이터의 z 선형 가속도가 설정한 하한값보다 작을 때:
            self.imu_linear_z_down = imu_linear_z  # 하한값을 현재 값으로 업데이트
        else:  # 그 외의 경우:
            pass  # 아무것도 하지 않음

        if imu_linear_z - self.imu_linear_z_down < 2 and self.imu_linear_z_up - imu_linear_z < 2:
            # z 선형 가속도의 범위가 상한값과 하한값 사이에 있을 때:
            motion = "GO"  # 로봇을 전진시키는 동작을 선택
        else:
            if self.imu_linear_z_ref - 2 < imu_linear_z < self.imu_linear_z_ref + 2:
                # IMU 데이터의 z 선형 가속도가 초기 값 주변에 있을 때:
                self.safe_num += 1  # 안전한 상태 확인 변수를 1 증가
            else:
                pass

            if 30 < self.safe_num:  # 안전한 상태 확인 변수가 30 이상일 때:
                motion = "GO"  # 로봇을 전진시키는 동작을 선택
                print("INIT")  # 초기화 메시지 출력
                self.safe_num = 0  # 안전한 상태 확인 변수 초기화
                self.imu_linear_z_down = self.imu_linear_z_ref  # 하한값 초기화
                self.imu_linear_z_up = self.imu_linear_z_ref  # 상한값 초기화
            else:
                motion = "STOP"  # 로봇을 정지시키는 동작을 선택
        return motion  # 선택한 동작 반환

    def act(self, motion):  # 실행 함수
        if motion == "GO":  # 전달받은 동작이 "GO"일 때:
            speed = 1  # 로봇 속도를 1로 설정
        else:  # 그 외의 경우:
            speed = 0  # 로봇 속도를 0으로 설정
        self.ctrl_msg.linear.x = speed  # Twist 메시지의 linear.x에 속도 설정
        self.pub.publish(self.ctrl_msg)  # 설정한 메시지를 "/cmd_vel" 토픽에 발행

    def run(self):  # 전체 총괄 함수
        if self.imu_flag == True:  # IMU 데이터가 수신된 경우:
            imu_linear_z = self.sense()  # IMU 데이터를 인지하여 변수에 저장
            motion = self.think(imu_linear_z)  # 판단 함수를 통해 동작 선택
            self.act(motion)  # 선택한 동작 실행
            print(f"{motion}({self.safe_num}): {imu_linear_z, self.imu_linear_z_down, self.imu_linear_z_up}")
        else:  # IMU 데이터가 수신되지 않은 경우:
            pass  # 아무 동작도 수행하지 않음
        self.rate.sleep()


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 대기합니다.
    while not rospy.is_shutdown():
        class_name.run()  # 클래스의 run 메서드를 호출하여 동작을 수행합니다.


if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()  # 메인 함수를 실행합니다.
