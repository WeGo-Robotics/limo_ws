#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 노드(Node)의 예제입니다.
# 이 예제를 통해 학생들은 클래스를 사용하여 ROS 노드를 구성하고 메시지를 구독하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(std_msgs.msg)을 가져옵니다.
import rospy
from std_msgs.msg import Int32


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/counter" 토픽에서 Int32 메시지를 구독하고, 콜백 함수(self.callback)를 호출합니다.
        self.sub = rospy.Subscriber("/counter", Int32, self.callback)  # ROS 2단계(필수): 서브스크라이버 설정

    def callback(self, msg):  # 3단계: 클래스 내의 콜백 함수 정의
        # 콜백 함수에서는 수신한 메시지를 처리하고 출력합니다.
        print(f"msg: {msg}")  # 출력: 메시지
        print(f"msg.data: {msg.data}")  # 출력: 메시지 항목(데이터)


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
    rospy.spin()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
