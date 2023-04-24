#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Draw:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_pub_color = rospy.Publisher("draw_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Set the height and width of the image
        height = 480
        width = 640

        # Define some colors
        white = [255, 255, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]
        cyan = [255, 255, 0]
        magenta = [255, 0, 255]
        yellow = [0, 255, 255]

        # Create an image with zeros
        img = np.zeros((height, width, 3), np.uint8)

        # Add ellipse to the image
        cv2.ellipse(img, (250, 200), (50, 25), 45, 0, 360, cyan, 2)

        # Convert the OpenCV image to a ROS image message
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

        # Publish the ROS image message
        self.image_pub_color.publish(img_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    draw = Draw()
    try:
        while not rospy.is_shutdown():
            draw.main()
    except rospy.ROSInterruptException:
        pass
