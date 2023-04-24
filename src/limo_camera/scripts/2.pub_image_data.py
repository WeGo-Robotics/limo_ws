#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Pub_Img_Data:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("camera_node")
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_CB)
        self.rgb_pub = rospy.Publisher("/camera_rgb_img", Image, queue_size=10)
        self.gray_pub = rospy.Publisher("/camera_gray_img", Image, queue_size=10)

    def camera_CB(self, data):
        rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        rgb_img_msg = self.bridge.cv2_to_imgmsg(rgb_img)
        gray_img_msg = self.bridge.cv2_to_imgmsg(gray_img)
        self.rgb_pub.publish(rgb_img_msg)
        self.gray_pub.publish(gray_img_msg)
        cv2.namedWindow("rgb_img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("gray_img", cv2.WINDOW_NORMAL)
        cv2.imshow("rgb_img", rgb_img)
        cv2.imshow("gray_img", gray_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    pub_img_data = Pub_Img_Data()
    rospy.spin()
