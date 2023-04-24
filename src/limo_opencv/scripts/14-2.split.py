#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os


# Define a class called split
class Split:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("split_data_node", anonymous=False)

        # Create publishers to publish the images
        self.image_pub_origin = rospy.Publisher("origin_img", Image, queue_size=1)
        self.image_pub_blue = rospy.Publisher("blue_img", Image, queue_size=1)
        self.image_pub_green = rospy.Publisher("green_img", Image, queue_size=1)
        self.image_pub_red = rospy.Publisher("red_img", Image, queue_size=1)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.img_path = self.file_path + "yosemite.jpg"

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
        img = np.array(
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
        img = cv2.imread(self.img_path, cv2.IMREAD_COLOR)
        b, g, r = cv2.split(img)

        # Convert the OpenCV images to ROS image messages
        ros_image_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        ros_image_blue = self.bridge.cv2_to_imgmsg(b)
        ros_image_green = self.bridge.cv2_to_imgmsg(g)
        ros_image_red = self.bridge.cv2_to_imgmsg(r)

        # Publish the ROS image messages
        self.image_pub_origin.publish(ros_image_img)
        self.image_pub_blue.publish(ros_image_blue)
        self.image_pub_green.publish(ros_image_green)
        self.image_pub_red.publish(ros_image_red)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    split = Split()
    try:
        while not rospy.is_shutdown():
            split.main()
    except rospy.ROSInterruptException:
        pass
