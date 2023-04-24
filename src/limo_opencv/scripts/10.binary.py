#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Binary:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_pub = rospy.Publisher("binary_img", Image, queue_size=10)

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
        # Read the image in BGR format using OpenCV
        wego_color = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)

        # Convert from color image to grayscale image
        wego_gray = cv2.cvtColor(wego_color, cv2.COLOR_BGR2GRAY)

        # Apply a binary threshold to the grayscale image to get a binary image
        _, wego_binary = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY)

        # Apply a binary inverse threshold to the grayscale image to get an inverted binary image
        _, wego_binary_INV = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY_INV)

        # Concatenate the binary and inverted binary images horizontally to create a comparison image
        con_binary = cv2.hconcat((wego_binary, wego_binary_INV))

        # Resize the comparison image to match the size of the grayscale image
        resize_con_binary = cv2.resize(
            con_binary, (wego_gray.shape[1], wego_gray.shape[0])
        )

        # Concatenate the grayscale image and the resized comparison image vertically
        thresh_rst = cv2.vconcat((wego_gray, resize_con_binary))

        # Convert the OpenCV image to a ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(thresh_rst)

        # Publish the ROS image message
        self.image_pub.publish(image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    binary = Binary()
    try:
        while not rospy.is_shutdown():
            binary.main()
    except rospy.ROSInterruptException:
        pass
