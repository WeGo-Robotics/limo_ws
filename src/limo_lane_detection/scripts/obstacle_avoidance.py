import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import *
from math import *


class Pub:
    def __init__(self):
        rospy.init_node("Pub_node")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.rate = rospy.Rate(10)
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.degrees = []
        self.degree_flag = False
        self.lidar_flag = False
        self.between_obstacle = 10

    def func(self):
        if self.lidar_flag == True:
            center_space = 0
            self.cmd_msg.linear.x = 0.1
            obstacle = []
            for index, value in enumerate(self.lidar_msg.ranges):
                if self.degree_flag == False:
                    self.degrees.append((self.lidar_msg.angle_min + self.lidar_msg.angle_increment * index) * 180 / pi)
                if 0 < value < 0.5:
                    obstacle.append(index)
                    if len(obstacle) >= 2:
                        if obstacle[-1] - obstacle[-2] > self.between_obstacle:
                            center_space = obstacle[-1] - obstacle[-2]
                            center_index = (obstacle[-1] + obstacle[-2]) // 2
                            # print("exist center space")
                        # else:
                        # print("no center space")
            self.degree_flag = True

            if len(obstacle) > 0:
                right_space = obstacle[0]
                left_space = len(self.lidar_msg.ranges) - obstacle[-1]
                seleted_space = max(left_space, center_space, right_space)

                if seleted_space == left_space:
                    print("left")
                    left_center_index = (len(self.lidar_msg.ranges) + obstacle[-1]) // 2
                    angular_z = self.degrees[left_center_index]
                elif seleted_space == right_space:
                    print("right")
                    right_center_index = (obstacle[0]) // 2
                    angular_z = self.degrees[right_center_index]
                else:
                    print("center")
                    angular_z = self.degrees[center_index]
            else:
                angular_z = 0.0

            self.cmd_msg.angular.z = angular_z * 0.01
            print(f"cmd_msg: {self.cmd_msg.angular.z}")
            self.pub.publish(self.cmd_msg)
            self.rate.sleep()
        else:
            print("cb_data 없음")

    def callback(self, msg):

        self.lidar_flag = True
        self.lidar_msg = msg


def main():
    pub = Pub()
    while not rospy.is_shutdown():
        pub.func()


if __name__ == "__main__":
    main()
