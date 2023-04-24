#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# Define a class for creating a color image and publishing it
class Color_Layer:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_pub = rospy.Publisher("color_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Define the color image as a numpy array
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]
        cyan = [255, 255, 0]
        magenta = [255, 0, 255]
        yellow = [0, 255, 255]
        color = blue
        color_img = np.array(
            [
                [blue, green, red, cyan, magenta, yellow],
                [green, red, cyan, magenta, yellow, blue],
                [red, cyan, magenta, yellow, red, green],
                [cyan, magenta, yellow, blue, green, red],
                [magenta, yellow, blue, green, red, cyan],
                [yellow, blue, green, red, cyan, magenta],
            ],
            np.uint8,
        )

        # Convert the OpenCV image to a ROS image message
        ros_image = self.bridge.cv2_to_imgmsg(color_img, "bgr8")

        # Publish the ROS image message
        self.image_pub.publish(ros_image)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    # Create an instance of the Color_Layer class
    color_layer = Color_Layer()
    try:
        while not rospy.is_shutdown():
            color_layer.main()
    except rospy.ROSInterruptException:
        pass
