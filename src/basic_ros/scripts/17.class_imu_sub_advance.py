#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 IMU(Inertial Measurement Unit) 데이터를 구독하고 해당 데이터를 쿼터니언(quaternion)에서 오일러 각도(euler angle)로 변환하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg)을 가져옵니다.
import rospy
from sensor_msgs.msg import Imu
from math import *


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/imu" 토픽에서 Imu 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        # 쿼터니언(quaternion)을 오일러 각도(euler angle)로 변환합니다.
        roll_x_rad, pitch_y_rad, yaw_z_rad = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll_x_degree = self.angle_to_degree(roll_x_rad)
        pitch_y_degree = self.angle_to_degree(pitch_y_rad)
        yaw_z_degree = self.angle_to_degree(yaw_z_rad)

        # 변환된 오일러 각도를 출력합니다.
        print(f"yaw_z_degree:{yaw_z_degree}")  # 출력 : 변수(yaw_z_degree)

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
        degree = angle * 180 / pi
        return degree


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
