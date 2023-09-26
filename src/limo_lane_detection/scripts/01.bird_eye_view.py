#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class Bird_Eye_View:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("bird_eye_node0")
        self.pub = rospy.Publisher(
            "/bird_eye/compressed", CompressedImage, queue_size=10
        )
        rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", CompressedImage, self.img_CB
        )

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

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        warp_img = self.img_warp(img)
        warp_img_msg = self.bridge.cv2_to_compressed_imgmsg(warp_img)
        self.pub.publish(warp_img_msg)
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("img_warp", cv2.WINDOW_NORMAL)
        cv2.imshow("img", img)
        cv2.imshow("img_warp", warp_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    bird_eye_view = Bird_Eye_View()
    rospy.spin()
