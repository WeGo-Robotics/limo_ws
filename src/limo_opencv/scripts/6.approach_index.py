#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# Define a class called Approach_Index
class Approach_Index:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create publishers to publish the images
        self.color_img_pub = rospy.Publisher("color_img", Image, queue_size=10)
        self.grayscale_img_pub = rospy.Publisher("gray_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Set the height and width of the images
        height = 150
        width = 150

        # Create a 3-channel black image with the specified height and width
        color_space = np.zeros((height, width, 3), np.uint8)

        # Create a single-channel black image with the specified height and width
        grayscale_space = np.zeros((height, width), np.uint8)

        # Define the colors blue, green and red as lists of RGB values
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]

        # Set a pixel in the center of the color image to blue
        color_space[75, 75] = blue

        # Set a pixel in the center of the color image to green
        color_space[0:50, 50:100] = green

        # Set a pixel in the center of the color image to red
        color_space[-10:-1, :] = red

        # Set a pixel in the center of the grayscale image to white
        grayscale_space[75, 75] = 255

        # Set a rectangular region in the grayscale image to a medium gray color
        grayscale_space[0:50, 50:100] = 150

        # Convert the OpenCV images to ROS image messages
        image_color = self.bridge.cv2_to_imgmsg(color_space, "bgr8")
        image_grayscale = self.bridge.cv2_to_imgmsg(grayscale_space)

        # Publish the ROS image message
        self.color_img_pub.publish(image_color)
        self.grayscale_img_pub.publish(image_grayscale)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    approach_index = Approach_Index()
    try:
        while not rospy.is_shutdown():
            approach_index.main()
    except rospy.ROSInterruptException:
        pass
