#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# Define a class called Color_And_Gray
class Color_And_Gray:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create publishers to publish the images
        self.image_pub_color = rospy.Publisher("bgr_img", Image, queue_size=10)
        self.image_pub_gray = rospy.Publisher("gray_img", Image, queue_size=10)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.wego_path = self.file_path + "wego.png"

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Read the image in BGR format using OpenCV
        wego_bgr = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)

        # Convert from color image to grayscale image
        wego_gray = cv2.cvtColor(wego_bgr, cv2.COLOR_BGR2GRAY)

        # Convert the OpenCV images to ROS image messages
        ros_image_color = self.bridge.cv2_to_imgmsg(wego_bgr, "bgr8")
        ros_image_gray = self.bridge.cv2_to_imgmsg(wego_gray)

        # Publish the ROS image message
        self.image_pub_color.publish(ros_image_color)
        self.image_pub_gray.publish(ros_image_gray)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    color_and_gray = Color_And_Gray()
    try:
        while not rospy.is_shutdown():
            color_and_gray.main()
    except rospy.ROSInterruptException:
        pass
