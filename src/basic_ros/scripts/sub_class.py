#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


class Subscribe_test:  # 1 단계
    def __init__(self):  # 2 단계
        rospy.init_node("wego_sub_node")
        rospy.Subscriber("/counter", Int32, self.callback)

    def callback(self, msg):  # 3 단계
        print(msg)


def main():  # 4 단계
    subscribe_test = Subscribe_test()
    rospy.spin()


if __name__ == "__main__":  # 5 단계
    main()
