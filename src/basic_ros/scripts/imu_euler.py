#!/usr/bin/env python3

# 모듈 가져오기
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from math import *
import os


# 클래스 생성
class E_STOP:
    # 초기화 및 초기 설정
    def __init__(self):
        # 노드 이름 설정
        rospy.init_node("wego_node")
        # 노드 역할 설정
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        rospy.Subscriber("/imu", Imu, self.imu_CB)
        imu_msg = Imu()
        self.degrees = [i for i in range(-180, 181, 45)]
        self.pre_time = rospy.get_time()
        self.ctrl_msg = Twist()
        self.pre_degree = 0

    # 함수 설정
    def imu_CB(self, msg):
        current_time = rospy.get_time()

        # print("current_time :",current_time)

        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(x, y, z, w)
        imu_degree = yaw_z * 180 / pi

        if current_time - self.pre_time > 10:
            self.pre_degree = self.pre_degree + 90
            if self.pre_degree > 180:
                self.pre_degree = self.pre_degree - 360
            else:
                pass
            self.pre_time = current_time

        if abs(self.pre_degree - imu_degree) < 3:
            steer = 0
        else:
            if self.pre_degree < imu_degree:
                steer = -(self.pre_degree - imu_degree) / 50
            else:
                steer = (self.pre_degree - imu_degree) / 50
        os.system("clear")
        print("pre_degree:", self.pre_degree)
        print("current_degree:", imu_degree)
        self.ctrl_msg.angular.z = steer
        self.ctrl_pub.publish(self.ctrl_msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


def main():
    e_stop = E_STOP()
    rospy.spin()


if __name__ == "__main__":
    main()
