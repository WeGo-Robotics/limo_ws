#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 노드(Node)의 예제입니다.
# 이 예제를 통해 학생들은 ROS 노드를 사용하여 메시지를 구독하고 처리하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(std_msgs.msg)을 가져옵니다.
import rospy
from std_msgs.msg import Int32

# ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의


# 콜백 함수(callback) 정의: 이 함수는 메시지를 수신할 때 호출됩니다.
def callback(msg):  # ROS 3단계(필수): 서브스크라이버 - 콜백 함수 설정
    print(f"msg: {msg}")  # 출력: 메시지
    print(f"msg.data: {msg.data}")  # 출력: 메시지 항목(데이터)


# ROS 서브스크라이버(Subscriber)를 설정합니다.
# "/counter" 토픽에서 Int32 메시지를 수신하고, 콜백 함수를 호출합니다.
sub = rospy.Subscriber("/counter", Int32, callback)  # ROS 2단계(필수): 서브스크라이버 설정

# rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
rospy.spin()  # 서브스크라이버: 수신 대기
