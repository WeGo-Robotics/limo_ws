#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from math import *
from time import *


class LKAS:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("LKAS_node")
        self.pub = rospy.Publisher("/sliding_windows/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.img_CB)
        self.ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.start_time = rospy.get_time()
        self.nothing_flag = False
        self.cmd_vel_msg = Twist()

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        yellow_lower = np.array([15, 80, 0])
        yellow_upper = np.array([45, 255, 255])

        # Define range of blend color in HSV
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 64, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Threshold the HSV image to get only white colors
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # Threshold the HSV image to get blend colors
        blend_mask = cv2.bitwise_or(yellow_mask, white_mask)
        blend_color = cv2.bitwise_and(img, img, mask=blend_mask)
        return blend_color

    def img_warp(self, img):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')

        img_size = [640, 480]
        # ROI
        src_side_offset = [0, 240]
        src_center_offset = [200, 315]
        src = np.float32(
            [
                [0, 479],
                [src_center_offset[0], src_center_offset[1]],
                [640 - src_center_offset[0], src_center_offset[1]],
                [639, 479],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [round(self.img_x * 0.125), 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0], self.img_y],
                [dst_offset[0], 0],
                [self.img_x - dst_offset[0], 0],
                [self.img_x - dst_offset[0], self.img_y],
            ]
        )
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, matrix, [self.img_x, self.img_y])
        return warp_img

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line

    def detect_nothing(self):
        self.nothing_left_x_base = round(self.img_x * 0.140625)
        self.nothing_right_x_base = self.img_x - round(self.img_x * 0.140625)

        self.nothing_pixel_left_x = np.array(np.zeros(self.nwindows) + round(self.img_x * 0.140625))

        self.nothing_pixel_right_x = np.array(np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.140625))

        self.nothing_pixel_y = np.array([round(self.window_height / 2) * index for index in range(0, self.nwindows)])

    def window_search(self, binary_line):
        # histogram을 생성합니다.
        # y축 기준 절반 아래 부분만을 사용하여 x축 기준 픽셀의 분포를 구합니다.
        bottom_half_y = binary_line.shape[0] / 2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스를 반환합니다.
        midpoint = np.int32(histogram.shape[0] / 2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint
        # show histogram
        # plt.hist(histogram)
        # plt.show()
        if left_x_base == 0:
            left_x_current = self.nothing_left_x_base
        else:
            left_x_current = left_x_base
        if right_x_base == midpoint:
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base

        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        ## window parameter
        # 적절한 윈도우의 개수를 지정합니다.
        nwindows = self.nwindows
        # 개수가 너무 적으면 정확하게 차선을 찾기 힘듭니다.
        # 개수가 너무 많으면 연산량이 증가하여 시간이 오래 걸립니다.
        window_height = self.window_height
        # 윈도우의 너비를 지정합니다. 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정합니다.
        margin = 80
        # 탐색할 최소 픽셀의 개수를 지정합니다.
        min_pix = round((margin * 2 * window_height) * 0.0031)

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        # pixel index를 담을 list를 만들어 줍니다.
        left_lane_idx = []
        right_lane_idx = []

        # Step through the windows one by one
        for window in range(nwindows):
            # window boundary를 지정합니다. (가로)
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            # print("check param : \n",window,win_y_low,win_y_high)

            # position 기준 window size
            win_x_left_low = left_x_current - margin
            win_x_left_high = left_x_current + margin
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            # window 시각화입니다.
            if left_x_current != 0:
                cv2.rectangle(
                    out_img,
                    (win_x_left_low, win_y_low),
                    (win_x_left_high, win_y_high),
                    (0, 255, 0),
                    2,
                )
            if right_x_current != midpoint:
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 0, 255),
                    2,
                )

            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index를 저장합니다.
            good_left_idx = (
                (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_left_low) & (lane_pixel_x < win_x_left_high)
            ).nonzero()[0]
            good_right_idx = (
                (lane_pixel_y >= win_y_low) & (lane_pixel_y < win_y_high) & (lane_pixel_x >= win_x_right_low) & (lane_pixel_x < win_x_right_high)
            ).nonzero()[0]

            # Append these indices to the lists
            left_lane_idx.append(good_left_idx)
            right_lane_idx.append(good_right_idx)

            # window내 설정한 pixel개수 이상이 탐지되면, 픽셀들의 x 좌표 평균으로 업데이트 합니다.
            if len(good_left_idx) > min_pix:
                left_x_current = np.int32(np.mean(lane_pixel_x[good_left_idx]))
            if len(good_right_idx) > min_pix:
                right_x_current = np.int32(np.mean(lane_pixel_x[good_right_idx]))

        # np.concatenate(array) => axis 0으로 차원 감소 시킵니다.(window개수로 감소)
        left_lane_idx = np.concatenate(left_lane_idx)
        right_lane_idx = np.concatenate(right_lane_idx)

        # window 별 좌우 도로 픽셀 좌표입니다.
        left_x = lane_pixel_x[left_lane_idx]
        left_y = lane_pixel_y[left_lane_idx]
        right_x = lane_pixel_x[right_lane_idx]
        right_y = lane_pixel_y[right_lane_idx]

        # 좌우 차선 별 2차함수 계수를 추정합니다.
        if len(left_x) == 0 and len(right_x) == 0:
            left_x = self.nothing_pixel_left_x
            left_y = self.nothing_pixel_y
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y
        else:
            if len(left_x) == 0:
                left_x = right_x - self.img_x / 2
                left_y = right_y
            elif len(right_x) == 0:
                right_x = left_x + self.img_x / 2
                right_y = left_y

        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2)
        # 좌우 차선 별 추정할 y좌표입니다.
        plot_y = np.linspace(0, binary_line.shape[0] - 1, 5)
        # 좌우 차선 별 2차 곡선을 추정합니다.
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
        center_fit_x = (right_fit_x + left_fit_x) / 2

        # # window안의 lane을 black 처리합니다.
        # out_img[lane_pixel_y[left_lane_idx], lane_pixel_x[left_lane_idx]] = (0, 0, 0)
        # out_img[lane_pixel_y[right_lane_idx], lane_pixel_x[right_lane_idx]] = (0, 0, 0)

        # 양쪽 차선 및 중심 선 pixel 좌표(x,y)로 변환합니다.
        center = np.asarray(tuple(zip(center_fit_x, plot_y)), np.int32)
        right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        left = np.asarray(tuple(zip(left_fit_x, plot_y)), np.int32)

        cv2.polylines(out_img, [left], False, (0, 0, 255), thickness=5)
        cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, left, right, center, left_x, left_y, right_x, right_y

    def meter_per_pixel(self):
        world_warp = np.array([[97, 1610], [109, 1610], [109, 1606], [97, 1606]], np.float32)
        meter_x = np.sum((world_warp[0] - world_warp[3]) ** 2)
        meter_y = np.sum((world_warp[0] - world_warp[1]) ** 2)
        meter_per_pix_x = meter_x / self.img_x
        meter_per_pix_y = meter_y / self.img_y
        return meter_per_pix_x, meter_per_pix_y

    def calc_curve(self, left_x, left_y, right_x, right_y):
        # WeGo simulation상의 차선의 간격(enu 좌표)을 통해 simulation상의 곡률을 구하는 함수입니다.
        # # Args:
        # left_x (np.array): 왼쪽 차선 pixel x값
        # left_y (np.array): 왼쪽 차선 pixel y값
        # right_x (np.array): 오른쪽 차선 pixel x값
        # right_y (np.array): 오른쪽 차선 pixel y값
        #
        # Returns:
        # float: 왼쪽, 오른쪽 차선의 곡률입니다.

        # 640p video/image, so last (lowest on screen) y index is 639
        y_eval = self.img_x - 1

        # Define conversions in x and y from pixels to meter
        # meter per pixel in each x, y dimension
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()

        # Fit new polynomials to x,y in world space(meterinate)
        left_fit_cr = np.polyfit(left_y * meter_per_pix_y, left_x * meter_per_pix_x, 2)
        right_fit_cr = np.polyfit(right_y * meter_per_pix_y, right_x * meter_per_pix_x, 2)
        # Calculate the new radius of curvature
        left_curve_radius = ((1 + (2 * left_fit_cr[0] * y_eval * meter_per_pix_y + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])

        right_curve_radius = ((1 + (2 * right_fit_cr[0] * y_eval * meter_per_pix_y + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

        return left_curve_radius, right_curve_radius

    def calc_vehicle_offset(self, sliding_window_img, left_x, left_y, right_x, right_y):
        # WeGo simulation상의 차선의 간격 meter을 통해 차량의 이탈정도를 구합니다.
        # Args:
        # sliding_window_img (_type_): _description_
        # left_x (np.array): 왼쪽 차선 pixel x 값
        # left_y (np.array): 왼쪽 차선 pixel y 값
        # right_x (np.array): 오른쪽 차선 pixel x 값
        # right_y (np.array): 오른쪽 차선 pixel y 값
        # Returns:
        # float: 차선 중앙으로부터 이탈정도를 확인합니다. (왼쪽 -, 오른쪽 +)

        # 좌우 차선 별 2차함수 계수 추정합니다.
        left_fit = np.polyfit(left_y, left_x, 2)
        right_fit = np.polyfit(right_y, right_x, 2)

        # Calculate vehicle center offset in pixels
        bottom_y = sliding_window_img.shape[0] - 1
        bottom_x_left = left_fit[0] * (bottom_y**2) + left_fit[1] * bottom_y + left_fit[2]
        bottom_x_right = right_fit[0] * (bottom_y**2) + right_fit[1] * bottom_y + right_fit[2]
        vehicle_offset = sliding_window_img.shape[1] / 2 - (bottom_x_left + bottom_x_right) / 2

        # Convert pixel offset to meter
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()
        vehicle_offset *= meter_per_pix_x

        return vehicle_offset

    def cam_cal_steer(self, left_curve_radius, right_curve_radius, vehicle_offset):
        curvature = 1 / ((left_curve_radius + right_curve_radius) / 2)
        cam_steer = (atan((1 * curvature) / 1 - (1 / 2) * curvature)) * 100
        if vehicle_offset > 0:
            # cam_steer = -atan(curvature)
            cam_steer = -cam_steer
        else:
            # cam_steer = atan(curvature)
            cam_steer = cam_steer
        return cam_steer

    def ctrl_cmd(self, vehicle_offset):
        self.cmd_vel_msg.linear.x = 0.05
        self.cmd_vel_msg.angular.z = -vehicle_offset * 0.1
        return self.cmd_vel_msg

    def img_CB(self, data):
        self.end_time = rospy.get_time()
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        self.nwindows = 10
        self.window_height = np.int32(img.shape[0] / self.nwindows)
        warp_img = self.img_warp(img)
        blend_img = self.detect_color(warp_img)
        binary_img = self.img_binary(blend_img)
        if self.nothing_flag == False:
            self.detect_nothing()
            self.nothing_flag = True
        (
            sliding_window_img,
            left,
            right,
            center,
            left_x,
            left_y,
            right_x,
            right_y,
        ) = self.window_search(binary_img)
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()
        left_curve_radius, right_curve_radius = self.calc_curve(left_x, left_y, right_x, right_y)
        vehicle_offset = self.calc_vehicle_offset(sliding_window_img, left_x, left_y, right_x, right_y)
        # cam_steer = self.cam_cal_steer(left_curve_radius, right_curve_radius, vehicle_offset)
        ctrl_cmd_msg = self.ctrl_cmd(vehicle_offset)
        if self.end_time - self.start_time >= 0.1:
            self.ctrl_pub.publish(ctrl_cmd_msg)
            self.start_time = self.end_time
        os.system("clear")
        print(f"------------------------------")
        print(f"left : {left}")
        print(f"right : {right}")
        print(f"center : {center}")
        print(f"left_x : {left_x}")
        print(f"left_y : {left_y}")
        print(f"right_x : {right_x}")
        print(f"right_y : {right_y}")
        print(f"meter_per_pix_x : {meter_per_pix_x}")
        print(f"meter_per_pix_y : {meter_per_pix_y}")
        print(f"left_curve_radius : {left_curve_radius}")
        print(f"right_curve_radius : {right_curve_radius}")
        print(f"vehicle_offset : {vehicle_offset}")
        # print(f"cam_steer : {cam_steer}")
        print(f"time : {self.start_time}")
        print(f"------------------------------")
        sliding_window_msg = self.bridge.cv2_to_compressed_imgmsg(sliding_window_img)
        self.pub.publish(sliding_window_msg)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("sliding_window_img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("sliding_window_img", sliding_window_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    lkas = LKAS()
    rospy.spin()
