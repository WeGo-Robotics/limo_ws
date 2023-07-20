#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


class Publish_test:  # 1 단계
    def __init__(self):  # 2 단계
        rospy.init_node("wego_pub_node")
        self.pub = rospy.Publisher("/counter", Int32, queue_size=1)
        self.num = 0
        self.rate = rospy.Rate(10)

    def run(self):  # 3 단계
        self.num += 1
        self.pub.publish(self.num)
        print(self.num)
        self.rate.sleep()


def main():  # 4 단계
    publish_test = Publish_test()
    while not rospy.is_shutdown():
        publish_test.run()


if __name__ == "__main__":  # 5 단계
    main()
