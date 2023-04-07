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
        self.pub = rospy.Publisher("/camera_rgb_image", Image, queue_size=10)

    def camera_CB(self, data):
        img_origin = self.bridge.imgmsg_to_cv2(data)
        img_origin = cv2.cvtColor(img_origin, cv2.COLOR_RGB2BGR)
        img_gray = cv2.cvtColor(img_origin, cv2.COLOR_BGR2GRAY)
        img_msg = self.bridge.cv2_to_imgmsg(img_origin)
        # img_msg = self.bridge.cv2_to_imgmsg(img_gray)
        self.pub.publish(img_msg)
        cv2.namedWindow("img_origin", cv2.WINDOW_NORMAL)
        cv2.namedWindow("img_gray", cv2.WINDOW_NORMAL)
        cv2.imshow("img_origin", img_origin)
        cv2.imshow("img_gray", img_gray)
        cv2.waitKey(1)


if __name__ == "__main__":
    pub_img_data = Pub_Img_Data()
    rospy.spin()
