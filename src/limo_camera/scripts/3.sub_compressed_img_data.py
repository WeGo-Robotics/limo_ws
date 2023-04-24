#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class Sub_Comp_Img_Data:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("camera_node")
        rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", CompressedImage, self.camera_CB
        )

    def camera_CB(self, data):
        comp_img = self.bridge.compressed_imgmsg_to_cv2(data)
        cv2.namedWindow("comp_img", cv2.WINDOW_NORMAL)
        cv2.imshow("comp_img", comp_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    sub_comp_img_data = Sub_Comp_Img_Data()
    rospy.spin()
