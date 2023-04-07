#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class Blend_Line_detect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("blend_line_node")
        self.pub = rospy.Publisher("/blend/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_CB)

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

    def img_warp(self, img, blend_color):
        # shape of img
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')
        # img_size = [640, 480]

        # ROI
        src_side_offset = [round(self.img_x * 0.046875), round(self.img_y * 0.208)]
        src_center_offset = [round(self.img_x * 0.14), round(self.img_y * 0.083)]
        src = np.float32(
            [
                [src_side_offset[0], self.img_y - src_side_offset[1]],
                [
                    self.img_x / 2 - src_center_offset[0],
                    self.img_y / 2 + src_center_offset[1],
                ],
                [
                    self.img_x / 2 + src_center_offset[0],
                    self.img_y / 2 + src_center_offset[1],
                ],
                [self.img_x - src_side_offset[0], self.img_y - src_side_offset[1]],
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
        blend_line_warp = cv2.warpPerspective(
            blend_color, matrix, [self.img_x, self.img_y]
        )
        return blend_line_warp

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        blend_color = self.detect_color(img)

        blend_line_warp = self.img_warp(img, blend_color)

        blend_line_msg = self.bridge.cv2_to_compressed_imgmsg(blend_line_warp)
        self.pub.publish(blend_line_msg)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("blend_line_warp", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("blend_line_warp", blend_line_warp)
        cv2.waitKey(1)


if __name__ == "__main__":
    blend_line_detect = Blend_Line_detect()
    rospy.spin()
