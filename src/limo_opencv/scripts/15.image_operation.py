#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os


# Define a class called Image_operation
class Image_operation:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_operation_node", anonymous=False)

        # Create a publisher to publish the image
        self.add_image_pub = rospy.Publisher("add_image_img", Image, queue_size=1)
        self.add_weighted_image_pub = rospy.Publisher(
            "add_weighted_image_img", Image, queue_size=1
        )
        self.subtrack_image_pub = rospy.Publisher(
            "subtrack_image_img", Image, queue_size=1
        )
        self.absdiff_image_pub = rospy.Publisher(
            "absdiff_image_img", Image, queue_size=1
        )

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.wego_path = self.file_path + "wego.png"
        self.hsv_path = self.file_path + "HSV.png"
        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Read the image
        wego_img = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        hsv_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)

        wego_img = cv2.resize(wego_img, (hsv_img.shape[1], hsv_img.shape[0]))
        # Image operation
        add_img = cv2.add(wego_img, hsv_img, dtype=cv2.CV_8U)
        add_weighted_img = cv2.addWeighted(wego_img, 0.5, hsv_img, 0.5, 0.0)
        subtrack_img = cv2.subtract(wego_img, hsv_img)
        absdiff_img = cv2.absdiff(wego_img, hsv_img)

        # Convert the OpenCV images to ROS image messages
        add_image_msg = self.bridge.cv2_to_imgmsg(add_img, "bgr8")
        add_weighted_image_msg = self.bridge.cv2_to_imgmsg(add_weighted_img, "bgr8")
        subtrack_image_msg = self.bridge.cv2_to_imgmsg(subtrack_img, "bgr8")
        absdiff_image_msg = self.bridge.cv2_to_imgmsg(absdiff_img, "bgr8")

        # Publish the ROS image messages
        self.add_image_pub.publish(add_image_msg)
        self.add_weighted_image_pub.publish(add_weighted_image_msg)
        self.subtrack_image_pub.publish(subtrack_image_msg)
        self.absdiff_image_pub.publish(absdiff_image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    image_operation = Image_operation()
    try:
        while not rospy.is_shutdown():
            image_operation.main()
    except rospy.ROSInterruptException:
        pass
