#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose


class Subscribe_test:  # 1 단계
    def __init__(self):  # 2 단계
        rospy.init_node("wego_sub_node")
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

    def callback(self, msg):  # 3 단계
        print(f"msg.x:{msg.x}")
        print(f"msg.y:{msg.y}")
        print(f"msg.theta:{msg.theta}")
        print(f"msg.linear_velocity:{msg.linear_velocity}")
        print(f"msg.angular_velocity:{msg.angular_velocity}")


def main():  # 4 단계
    subscribe_test = Subscribe_test()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
