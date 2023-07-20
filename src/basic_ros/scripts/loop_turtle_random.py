#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from random import *


class Subscribe_test:  # 1 단계
    def __init__(self):  # 2 단계
        rospy.init_node("wego_node")
        self.turtle_pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=3)
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

    def callback(self, msg):  # 3 단계
        cmd_msg = Twist()
        cmd_msg.linear.x = random() * 2  # 0 이상 ~ 2 미만 의 값
        cmd_msg.angular.z = random() * 4 - 2  # -2 ~ 2 미만 의 값
        print(f"x:{msg.x}")
        print(f"y:{msg.y}")
        print(f"theta:{msg.theta}")
        print(f"linear_velocity:{msg.linear_velocity}")
        print(f"angular_velocity:{msg.angular_velocity}")
        self.turtle_pub.publish(cmd_msg)


def main():  # 4 단계
    subscribe_test = Subscribe_test()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
