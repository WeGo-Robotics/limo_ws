#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from math import *
import cv2
import numpy as np
from cv_bridge import CvBridge


class Class_sub:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node 이름 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb)  # 2. node 역할 설정
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_cb)  # 2. node 역할 설정
        self.lidar_msg = LaserScan()
        self.camera_msg = CompressedImage()
        self.cmd_msg = Twist()
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.lidar_flag = False
        self.camera_flag = False
        self.e_stop_flag = False
        self.steer = 0

    def lidar_cb(self, msg):  # 3. subscriber - callback
        if msg != None:
            self.lidar_msg = msg
            self.lidar_flag = True
        else:
            self.lidar_flag = False

    def camera_cb(self, msg):
        if msg != None:
            self.camera_msg = msg
            self.camera_flag = True
        else:
            self.lidar_flag = False

    def e_stop(self):
        if self.lidar_flag == True:
            degree_min = self.lidar_msg.angle_min * 180 / pi
            degree_max = self.lidar_msg.angle_max * 180 / pi
            degree_increment = self.lidar_msg.angle_increment * 180 / pi
            degrees = [degree_min + degree_increment * index for index, value in enumerate(self.lidar_msg.ranges)]
            self.obstacle = 0
            for index, value in enumerate(self.lidar_msg.ranges):
                if 0 < value < 0.5 and abs(degrees[index]) < 30:
                    print(f"장애물:{degrees[index]}")
                    self.obstacle = self.obstacle + 1
                else:
                    pass

            if self.obstacle > 0:
                self.e_stop_flag = True
            else:
                self.e_stop_flag = False

    def lkas(self):
        if self.camera_flag == True:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(self.camera_msg)
            y, x, channel = cv_img.shape
            hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

            white_lower = np.array([0, 0, 150])
            white_upper = np.array([179, 20, 255])
            white_filter = cv2.inRange(hsv_img, white_lower, white_upper)

            yellow_lower = np.array([15, 80, 80])
            yellow_upper = np.array([45, 255, 255])
            yellow_filter = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
            combine_filter = cv2.bitwise_or(white_filter, yellow_filter)

            and_img = cv2.bitwise_and(cv_img, cv_img, mask=yellow_filter)

            margin_x = 250
            margin_y = 300

            src_pt1 = (0, y)
            src_pt2 = (margin_x, margin_y)
            src_pt3 = (x - margin_x, margin_y)
            src_pt4 = (x, y)
            src_pts = np.float32([src_pt1, src_pt2, src_pt3, src_pt4])

            dst_margin_x = 120

            dst_pt1 = (dst_margin_x, y)
            dst_pt2 = (dst_margin_x, 0)
            dst_pt3 = (x - dst_margin_x, 0)
            dst_pt4 = (x - dst_margin_x, y)
            dst_pts = np.float32([dst_pt1, dst_pt2, dst_pt3, dst_pt4])

            matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            matrix_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
            warp_img = cv2.warpPerspective(and_img, matrix, (x, y))
            gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
            bin_img = np.zeros_like(gray_img)
            bin_img[gray_img != 0] = 1
            center_index = x // 2

            window_num = 8
            margin = 40
            window_y_size = y // window_num  # 60
            left_indices = []
            right_indices = []

            for i in range(0, window_num):
                upper_y = y - window_y_size * (i + 1)
                lower_y = y - window_y_size * i

                left_window = bin_img[upper_y:lower_y, :center_index]
                left_histogram = np.sum(left_window, axis=0)
                left_histogram[left_histogram < 40] = 0

                right_window = bin_img[upper_y:lower_y, center_index:]
                right_histogram = np.sum(right_window, axis=0)
                right_histogram[right_histogram < 40] = 0

                try:
                    left_nonzero = np.nonzero(left_histogram)[0]
                    left_avg_index = (left_nonzero[0] + left_nonzero[-1]) // 2
                    left_indices.append(left_avg_index)

                    right_nonzero = np.nonzero(right_histogram)[0]
                    right_avg_index = (right_nonzero[0] + right_nonzero[-1]) // 2 + center_index
                    right_indices.append(right_avg_index)

                    cv2.line(
                        warp_img,
                        (left_avg_index, y - window_y_size * (i + 1) + window_y_size // 2),
                        (left_avg_index, y - window_y_size * (i + 1) + window_y_size // 2),
                        (0, 0, 255),
                        10,
                    )
                    cv2.line(
                        warp_img,
                        (right_avg_index, y - window_y_size * (i + 1) + window_y_size // 2),
                        (right_avg_index, y - window_y_size * (i + 1) + window_y_size // 2),
                        (255, 0, 0),
                        10,
                    )
                    cv2.rectangle(warp_img, (left_avg_index - margin, upper_y), (left_avg_index + margin, lower_y), (255, 0, 0), 3)
                    cv2.rectangle(warp_img, (right_avg_index - margin, upper_y), (right_avg_index + margin, lower_y), (0, 0, 255), 3)
                    left_avg_indices = np.average(left_indices)
                    right_avg_indices = np.average(right_indices)
                    avg_indices = int((left_avg_indices + right_avg_indices) // 2)
                except:
                    pass

            try:
                cv2.line(warp_img, (avg_indices, 0), (avg_indices, y), (0, 255, 255), 3)
                error_index = center_index - avg_indices
                self.steer = error_index * pi / x
            except:
                pass
            cv2.line(warp_img, (center_index, 0), (center_index, y), (0, 255, 0), 3)
            warp_inv_img = cv2.warpPerspective(warp_img, matrix_inv, (x, y))

            cv2.circle(cv_img, src_pt1, 20, (255, 0, 0), -1)
            cv2.circle(cv_img, src_pt2, 20, (0, 255, 0), -1)
            cv2.circle(cv_img, src_pt3, 20, (0, 0, 255), -1)
            cv2.circle(cv_img, src_pt4, 20, (0, 255, 255), -1)
            cv2.imshow("cv_img", cv_img)
            cv2.imshow("and_img", and_img)
            # cv2.imshow("combine_filter",combine_filter)
            cv2.imshow("warp_img", warp_img)
            cv2.imshow("warp_inv_img", warp_inv_img)
            # cv2.imshow("gray_img",gray_img)
            # cv2.imshow("bin_img",bin_img)

            cv2.waitKey(1)

    def ctrl(self):
        if self.e_stop_flag == True:
            self.cmd_msg.linear.x = 0
            self.cmd_msg.angular.z = 0
        else:
            self.cmd_msg.linear.x = 0.1
            self.cmd_msg.angular.z = self.steer
        print(self.steer)
        self.pub.publish(self.cmd_msg)
        self.rate.sleep()


class_sub = Class_sub()
while not rospy.is_shutdown():
    class_sub.e_stop()
    class_sub.lkas()
    class_sub.ctrl()
