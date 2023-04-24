#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Pub_Img_Data:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("camera_node")
        rospy.Subscriber("/camera/depth/image_raw", Image, self.camera_CB)
        self.pub = rospy.Publisher("/camera_depth_image", Image, queue_size=10)

    def camera_CB(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        # print(img)
        _, img_binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
        img_binary_array = np.array(img_binary, dtype=np.dtype("f8"))
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        img_normalize = cv2.normalize(
            img_binary_array, img_binary_array, 0, 1, cv2.NORM_MINMAX
        )
        img_msg = self.bridge.cv2_to_imgmsg(img_normalize, "64FC1")
        self.pub.publish(img_msg)
        cv2.namedWindow("depth_img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("depth_img_normalize", cv2.WINDOW_NORMAL)
        cv2.imshow("depth_img", img)
        cv2.imshow("depth_img_normalize", img_normalize)
        cv2.waitKey(1)


if __name__ == "__main__":
    pub_img_data = Pub_Img_Data()
    rospy.spin()
