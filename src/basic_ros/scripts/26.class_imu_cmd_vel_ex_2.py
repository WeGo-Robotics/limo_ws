#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 IMU(Inertial Measurement Unit) 센서를 활용하여 로봇을 제어하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 수학 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from math import *


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # 1단계: 노드의 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/imu" 토픽에서 Imu 메시지를 구독하고, 콜백 함수(imu_CB)를 호출합니다.
        self.sub = rospy.Subscriber("/imu", Imu, self.imu_CB)  # 2단계: 노드의 역할 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/cmd_vel" 토픽에 Twist 메시지를 발행합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2단계: 노드의 역할 설정

        self.imu_msg = Imu()  # Imu 메시지 타입 설정 및 초기화
        self.ctrl_msg = Twist()  # Twist 메시지 타입 설정 및 초기화

        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정

        self.imu_flag = False  # 메세지 수신 확인을 위한 변수 설정
        self.degree_num = 0
        self.degree = 0
        self.timer = rospy.Timer(rospy.Duration(5), self.update_degree)

    def imu_CB(self, msg):
        if msg != -1:  # 메세지가 들어 왔을 시,
            self.imu_msg = msg  # 메세지를 self.imu_msg에 저장
            self.imu_flag = True  # 메세지 수신 확인 변수를 True로 저장
        else:
            self.imu_flag = False  # 메세지 수신 확인 변수를 False로 저장

    def quaternion_to_euler(self, x, y, z, w):
        """
        쿼터니언을 오일러 각도(roll, pitch, yaw)로 변환
        롤은 라디안 단위로 x를 중심으로 회전합니다(시계 반대 방향)
        피치는 라디안 단위로 y를 중심으로 회전합니다(시계 반대 방향)
        yaw는 zin 라디안(반시계 방향)을 중심으로 회전합니다
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # 라디안 단위로 반환

    def angle_to_degree(self, angle):
        degree = angle * 180 / pi  # 라디안을 도(degree)로 변환
        return degree

    def update_degree(self, event):
        degrees = [-90, 0, 90, 180]  # 변경할 각도 목록
        self.degree = degrees[self.degree_num]  # 현재 각도 설정
        self.degree_num += 1  # 다음 각도로 변경
        self.degree_num = self.degree_num % 4  # 인덱스 순환

    def sense(self):  # 인지 함수
        w = self.imu_msg.orientation.w
        x = self.imu_msg.orientation.x
        y = self.imu_msg.orientation.y
        z = self.imu_msg.orientation.z

        # 쿼터니언을 오일러 각도로 변환하여 로봇의 현재 방향을 감지
        roll_x, pitch_y, yaw_z = self.quaternion_to_euler(x, y, z, w)
        yaw_z_deg = self.angle_to_degree(yaw_z)  # 감지한 각도를 도(degree)로 변환
        return yaw_z_deg  # 로봇의 현재 방향을 감지한 각도로 반환

    def think(self, yaw_z_deg):  # 판단 함수
        if self.degree - 1 < yaw_z_deg < self.degree + 1:
            motion = "STOP"  # 로봇의 현재 방향과 목표 각도가 거의 일치하는 경우 정지 명령
        else:
            if yaw_z_deg < self.degree:
                motion = "LEFT"  # 로봇의 현재 방향이 목표 각도보다 작은 경우 왼쪽으로 회전 명령
            else:
                motion = "RIGHT"  # 로봇의 현재 방향이 목표 각도보다 큰 경우 오른쪽으로 회전 명령
        return motion  # 변수(motion)값 리턴

    def act(self, motion):  # 제어 함수
        if motion == "STOP":  # 변수(motion)의 값이 "STOP"인 경우:
            steer = 0  # 변수(steer)에 0으로 저장 (정지)
        else:  # 변수(motion)의 값이 "LEFT" 또는 "RIGHT"인 경우:
            if motion == "LEFT":  # 변수(motion)의 값이 "LEFT"인 경우:
                steer = -1  # 변수(steer)에 -1로 저장 (왼쪽으로 회전)
            else:
                steer = 1  # 변수(steer)에 1로 저장 (오른쪽으로 회전)

        # Twist 메시지를 사용하여 로봇의 움직임을 설정
        self.ctrl_msg.angular.z = steer  # 변수(steer)의 값을 self.ctrl_msg.angular.z에 저장
        self.pub.publish(self.ctrl_msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # 주기 설정에 따라 일정한 시간 동안 대기

    def run(self):  # 전체 총괄 함수
        if self.imu_flag == True:  # 메세지 수신 확인 변수(self.imu_flag)가 True인 경우:
            yaw_z_deg = self.sense()  # 인지 함수(self.sense)를 통해 로봇의 현재 방향을 감지
            motion = self.think(yaw_z_deg)  # 판단 함수(self.think)를 통해 로봇의 움직임을 결정
            self.act(motion)  # 제어 함수(self.act)를 통해 로봇을 움직임
        else:  # 메세지 수신 확인 변수(self.imu_flag)가 False인 경우:
            pass  # 아무 동작도 하지 않고 패스합니다.


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.
    while not rospy.is_shutdown():
        class_name.run()  # ROS 노드를 실행하고 메시지 수신을 대기합니다.


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
