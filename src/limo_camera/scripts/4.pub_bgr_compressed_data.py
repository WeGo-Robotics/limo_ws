#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class Pub_Comp_Img_Data:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("camera_node")
        rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", CompressedImage, self.camera_CB
        )
        self.img_bgr_pub = rospy.Publisher(
            "/camera_bgr_img/compressed/",
            CompressedImage,
            queue_size=10,
        )
        self.img_gray_pub = rospy.Publisher(
            "/camera_gray_img/compressed/",
            CompressedImage,
            queue_size=10,
        )

    def camera_CB(self, data):
        comp_img_bgr = self.bridge.compressed_imgmsg_to_cv2(data)
        comp_img_gray = cv2.cvtColor(comp_img_bgr, cv2.COLOR_BGR2GRAY)
        comp_img_bgr_msg = self.bridge.cv2_to_compressed_imgmsg(comp_img_bgr)
        comp_img_gray_msg = self.bridge.cv2_to_compressed_imgmsg(comp_img_gray)
        self.img_bgr_pub.publish(comp_img_bgr_msg)
        self.img_gray_pub.publish(comp_img_gray_msg)
        cv2.namedWindow("comp_img_bgr", cv2.WINDOW_NORMAL)
        cv2.namedWindow("comp_img_gray", cv2.WINDOW_NORMAL)
        cv2.imshow("comp_img_bgr", comp_img_bgr)
        cv2.imshow("comp_img_gray", comp_img_gray)
        cv2.waitKey(1)


if __name__ == "__main__":
    pub_comp_img_Data = Pub_Comp_Img_Data()
    rospy.spin()
