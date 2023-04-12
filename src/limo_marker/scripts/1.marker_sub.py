#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


# 클래스 생성
class Alvar_Marker_Sub:
    # 클래스 속성 정의
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("alvar_node", anonymous=True)
        # /ar_pose_marker 토픽으로부터 AlvarMarkers 메시지를 수신하는 Subscriber 생성
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)

    # 클래스의 메서드 정의
    def marker_CB(self, data):
        # data 객체의 markers 출력. AlvarMarkers 메시지 객체에서 AR 마커의 정보를 화면에 출력
        print(data.markers)


if __name__ == "__main__":
    # 클래스 객체 생성
    alvar_marker_sub = Alvar_Marker_Sub()
    try:
        # 함수를 호출하여 ROS 노드를 실행하고, 메시지를 수신하는 동안 무한 루프
        rospy.spin()
    # 예외가 발생하면, 예외를 무시하고 프로그램 종료
    except rospy.ROSInterruptException:
        pass
