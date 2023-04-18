#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class CvtColor:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.wego_path = self.file_path + "wego.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Read the image
        color_img = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)

        # Convert the image
        cvt_gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        cvt_hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        cvt_hsl = cv2.cvtColor(color_img, cv2.COLOR_BGR2HLS)
        cvt_lab = cv2.cvtColor(color_img, cv2.COLOR_BGR2Lab)

        # Combine the images
        gray_3d = np.dstack((cvt_gray, cvt_gray, cvt_gray))
        cvt1 = cv2.hconcat([gray_3d, cvt_lab])
        cvt2 = cv2.hconcat([cvt_hsv, cvt_hsl])
        rst = cv2.vconcat([cvt1, cvt2])
        dst = cv2.resize(rst, (2000, 1000), interpolation=cv2.INTER_LINEAR)

        # Convert the OpenCV image to a ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(dst, "bgr8")

        # Publish the ROS image message
        self.image_cvt_pub.publish(image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    cvtColor = CvtColor()
    try:
        while not rospy.is_shutdown():
            cvtColor.main()
    except rospy.ROSInterruptException:
        pass
