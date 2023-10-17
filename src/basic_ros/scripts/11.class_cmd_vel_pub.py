#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 클래스를 사용하여 로봇을 제어하고 움직이는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(geometry_msgs.msg)을 가져옵니다.
import rospy
from geometry_msgs.msg import Twist


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_pub_node"로 지정됩니다.
        rospy.init_node("wego_pub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/cmd_vel" 토픽에 Twist 메시지를 발행하는 퍼블리셔를 만듭니다.
        # "queue_size"는 메시지 대기열 크기를 나타냅니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # ROS 2단계(필수): 퍼블리셔 설정

        # 메시지 발행 주기를 설정합니다. 이 예제에서는 10Hz로 설정합니다.
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 발행 주기 설정

        # Twist 메시지 타입의 메시지 객체를 생성하고 초기화합니다.
        self.msg = Twist()  # 메시지 타입 설정 및 초기화

    def func(self):  # 3단계: 클래스 내의 함수 설정
        # 메시지 항목(linear.x)을 1로 설정합니다.
        self.msg.linear.x = 1  # 메시지 항목(linear.x) - 데이터 값 변경

        # 퍼블리셔를 사용하여 Twist 메시지를 발행합니다.
        self.pub.publish(self.msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행

        # 현재 메시지와 linear.x 값을 출력합니다.
        print(f"self.msg: {self.msg}")  # 출력: 메시지
        print(f"self.msg.linear.x: {self.msg.linear.x}")  # 출력: 메시지(linear.x)

        # 지정한 발행 주기에 따라 슬립합니다. 이것은 메시지를 일정한 주기로 발행하기 위해 사용됩니다.
        self.rate.sleep()  # ROS 3-1단계(옵션): 퍼블리셔 - 주기 실행


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # 노드가 종료되지 않는 한 퍼블리셔를 실행하고 메시지를 발행합니다.
    while not rospy.is_shutdown():
        class_name.func()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
