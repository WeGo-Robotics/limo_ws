#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 IMU(Inertial Measurement Unit) 데이터를 구독하고 해당 데이터를 수정하여 다시 발행하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg)을 가져옵니다.
import rospy
from sensor_msgs.msg import Imu


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/imu2" 토픽에 Imu 메시지를 발행합니다.
        self.pub = rospy.Publisher("/imu2", Imu, queue_size=3)  # ROS 2단계: 노드 역할 - 퍼블리셔 설정

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/imu" 토픽에서 Imu 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        rospy.Subscriber("/imu", Imu, self.callback)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        self.rate = rospy.Rate(100)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정
        self.imu_msg = Imu()  # 메세지 타입 설정 및 초기화

    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 설정
        print(f"msg : {msg}")  # 메세지 읽기
        print(f"msg.orientation : {msg.orientation}")  # 출력: 메세지 항목(orientation)
        print(f"msg.linear_acceleration : {msg.linear_acceleration}")  # 출력: 메세지 항목(linear_acceleration)
        print(f"msg.angular_velocity : {msg.angular_velocity}")  # 출력: 메세지 항목(angular_velocity)

        # 콜백 함수 내에서 IMU 데이터를 수정합니다.
        self.imu_msg.header = msg.header  # 메세지 항목(header) - 데이터 값 변경
        self.imu_msg.orientation.x = 1  # 메세지 항목(orientation.x) - 데이터 값 변경
        self.imu_msg.orientation.y = 1  # 메세지 항목(orientation.y) - 데이터 값 변경

        # 수정한 데이터를 다시 발행합니다.
        self.pub.publish(self.imu_msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행
        self.rate.sleep()  # ROS 3-1단계(옵션): 퍼블리셔 - 주기 실행


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
