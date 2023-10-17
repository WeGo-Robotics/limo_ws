#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 사용하는 Python 노드(Node)의 기본 구조를 보여주는 예제입니다.
# 이 예제를 통해 학생들은 ROS 노드와 클래스의 개념을 이해하고, ROS 노드를 만들 때의 기본적인 구조를 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(std_msgs.msg)을 가져옵니다.
import rospy
from std_msgs.msg import *


# Class_Name 클래스 정의: 이 클래스는 노드의 동작을 구체적으로 정의합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        pass

    def func(self):  # 3단계: 클래스 내의 함수 정의
        pass


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # Class_Name 클래스의 인스턴스를 생성합니다.

    # ROS가 종료될 때까지 아래의 코드를 반복 실행합니다.
    while not rospy.is_shutdown():
        class_name.func()  # Class_Name 클래스의 func() 함수를 실행합니다.


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
