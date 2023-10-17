#!/usr/bin/env python3

# 위의 코드는 ROS(로봇 운영 체제)에서 동작하는 Python 클래스 예제입니다.
# 이 예제를 통해 학생들은 LIDAR(Laser Range Finder) 데이터를 활용하여 로봇을 제어하는 방법을 배울 수 있습니다.

# rospy 라이브러리와 필요한 메시지 유형(sensor_msgs.msg,geometry_msgs.msg) 및 수학 라이브러리를 가져옵니다.
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import *


# Class_Name 클래스 정의: ROS 노드를 클래스로 정의하여 코드를 구조화합니다.
class Class_Name:  # 1단계: 클래스 이름 정의
    def __init__(self):  # 2단계: 클래스 초기화 및 초기 설정
        # ROS 노드를 초기화합니다. 노드 이름은 "wego_sub_node"로 지정됩니다.
        rospy.init_node("wego_sub_node")  # ROS 1단계(필수): 노드 이름 정의

        # ROS 서브스크라이버(Subscriber)를 설정합니다.
        # "/scan" 토픽에서 LaserScan 메시지를 구독하고, 콜백 함수(lidar_CB)를 호출합니다.
        rospy.Subscriber("/scan", LaserScan, self.lidar_CB)  # ROS 2단계: 노드 역할 - 서브스크라이버 설정

        # ROS 퍼블리셔(Publisher)를 설정합니다.
        # "/cmd_vel" 토픽에 Twist 메시지를 발행합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # ROS 2단계: 노드 역할 - 퍼블리셔 설정

        # 메시지 타입 설정 및 초기화
        self.laser_msg = LaserScan()
        self.ctrl_msg = Twist()

        # ROS 퍼블리셔 주기 설정
        self.rate = rospy.Rate(10)  # ROS 2-1단계(옵션): 퍼블리셔 - 주기 설정

        # 메세지 수신 확인을 위한 변수 설정
        self.laser_flag = False

        # 인덱스를 각도로 저장할 행렬 설정
        self.degrees = []

        # 저장할 행렬 확인을 위한 변수 설정
        self.degrees_flag = False

    def lidar_CB(self, msg):
        if msg != -1:  # 메세지가 들어 왔을 시,
            self.laser_msg = msg  # 메세지를 self.laser_mag에 저장
            self.laser_flag = True  # 메세지 수신 확인 변수를 True로 저장
        else:
            self.laser_flag = False  # 메세지 수신 확인 변수를 False로 저장

    def sense(self):  # 인지 함수
        if self.laser_flag == True:
            if self.laser_flag == False:  # 변수 self.laser_flag가 False인 경우 :
                self.degrees = [(self.laser_msg.angle_min + self.laser_msg.angle_increment * i) * 180 / pi for i, v in enumerate(self.laser_msg.ranges)]
                # 최초 각도 값에서 msg.ranges의 인덱스 * 각도 증가량 씩 더하여 self.degrees로 저장
                self.laser_flag = True  # 변수 self.laser_flag가 True로 저장
        return self.degrees

    def think(self, degrees):  # 판단 함수
        object_degrees = []  # 검출된 장애물의 각도를 object_degrees에 저장
        for index, value in enumerate(self.laser_msg.ranges):  # self.laser_msg.ranges의 index와 value를 반복하여 추출한다.
            if 0 < self.laser_msg.ranges[index] < 0.30 and abs(degrees[index]) < 91:
                # self.laser_msg.ranges[index](범위)가 0에서 0.30 사이이며, abs(degrees[index])(절대값 각도)가 30도 이하일 때 :
                object_degrees.append(index)  # object_degrees에 index를 추가.
            else:  # 이외의 경우 :
                pass  # 패스

        if len(object_degrees) == 0:  # 장애물이 측정된 각도의 행렬의 크기가 0 인 경우:
            motion = "STRAIGHT"  # 변수(motion)에 "STRAIGHT"으로 저장
        else:
            right = object_degrees[0]
            left = len(self.degrees) - object_degrees[-1]
            if right > left:
                motion = "RIGHT"  # 변수(motion)에 "RIGHT"으로 저장
            else:  # 이외의 경우 :
                motion = "LEFT"  # 변수(motion)에 "LEFT"으로 저장
        return motion  # 변수(motion)값 리턴

    def act(self, motion):  # 제어 함수
        if motion == "STRAIGHT":  # 변수(motion)의 값이 "STRAIGHT"인 경우:
            steer = 0  # 변수(steer)에 0으로 저장
        else:  # 변수(motion)의 값이 "LEFT"인 경우:
            if motion == "RIGHT":  # 변수(motion)의 값이 "RIGHT"인 경우:
                steer = 1  # 변수(steer)에 1으로 저장
            else:
                steer = -1  # 변수(steer)에 -1으로 저장
        self.ctrl_msg.angular.z = steer  # 변수(steer)의 값을 self.ctrl_msg.linear.x에 저장
        self.pub.publish(self.ctrl_msg)  # ROS 3단계(필수): 퍼블리셔 - 퍼블리시 실행

    def run(self):  # 전체 총괄 함수
        if self.laser_flag == True:  # 변수(self.laser_flag)가 True인 경우:
            degrees = self.sense()  # 함수(self.sense)를 통해 출력된 값을 변수(degrees)로 저장
            motion = self.think(degrees)  # 함수(self.think)에 변수(degrees)를 입력하여 출력된 값을 변수(motion)으로 저장
            self.act(motion)  # 함수(self.act)에 변수(motion)을 입력
        else:  # 이외의 경우 :
            pass  # 패스
        self.rate.sleep()


# 메인 함수 정의: ROS 노드를 실행하기 위한 메인 함수입니다.
def main():  # 4단계: 메인 함수 정의
    class_name = Class_Name()  # 클래스(Class_Name)를 변수(class_name)에 저장
    while not rospy.is_shutdown():
        class_name.run()


# 직접 실행 코드: 스크립트가 직접 실행될 때 main() 함수를 호출합니다.
if __name__ == "__main__":  # 5단계: 직접 실행 구문 정의
    main()
