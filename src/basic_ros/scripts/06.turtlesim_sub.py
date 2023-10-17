#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 노드(Node)의 예제입니다.
# 이 예제를 통해 학생들은 서브스크라이버를 사용하여 로봇의 위치 정보를 구독하고 출력하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(turtlesim.msg)을 가져옵니다.
import rospy
from turtlesim.msg import Pose

# ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

# ROS 서브스크라이버(Subscriber)를 설정합니다.
# "/turtle1/pose" 토픽에서 Pose 메시지를 구독하고, 콜백 함수(callback)를 호출합니다.
sub = rospy.Subscriber("/turtle1/pose", Pose, callback)  # ROS 2단계(필수): 서브스크라이버 설정

# rospy.spin() 함수를 호출하여 노드를 실행하고 메시지 수신을 계속 대기합니다.
rospy.spin()


# 서브스크라이버의 콜백 함수: 수신한 메시지를 처리하고 출력합니다.
def callback(msg):  # 3단계: 서브스크라이버 - 콜백 함수 정의
    # 수신한 메시지와 메시지 항목(x, y)을 출력합니다.
    print(f"msg: {msg}")  # 출력: 메시지
    print(f"msg.x: {msg.x}")  # 출력: 메시지 항목(x)
    print(f"msg.y: {msg.y}")  # 출력: 메시지 항목(y)
