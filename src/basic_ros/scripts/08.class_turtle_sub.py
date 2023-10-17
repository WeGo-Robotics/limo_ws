#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 클래스를 사용하여 로봇의 위치 정보를 구독하고 출력하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(turtlesim.msg)을 가져옵니다.
import rospy
from turtlesim.msg import Pose


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/turtle1/pose" 토픽에서 Pose 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.callback)  # ROS 2단계(필수): 서브스크라이버 설정

    def callback(self, msg):  # 3단계: 클래스 내의 함수 설정
        # 수신한 메시지와 메시지 항목(x, y)을 출력합니다.
        print(f"msg: {msg}")  # 출력: 메시지
        print(f"msg.x: {msg.x}")  # 출력: 메시지 항목(x)
        print(f"msg.y: {msg.y}")  # 출력: 메시지 항목(y)


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
