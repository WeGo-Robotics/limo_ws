#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

# 클래스 생성
class VLP16: 

# 초기화 및 초기 설정 
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        rospy.Subscriber("/velodyne_points",PointCloud2,self.vlp_CB)
        self.vlp_msg = PointCloud2()
        
    def vlp_CB(self,msg):
        self.vlp_msg = msg
        print(self.vlp_msg.header.frame_id)
    
def main():
    vlp16 = VLP16()
    rospy.spin()

if __name__=="__main__":
    main()