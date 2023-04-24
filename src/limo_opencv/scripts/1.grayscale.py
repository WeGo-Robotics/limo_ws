#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# Create a class called Gray_Scale
class Gray_Scale:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_pub = rospy.Publisher("gray_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

    # Define the main function of the class
    def main(self):
        # Define image to be published
        gray = np.array(
            [
                [0, 255, 127, 255, 0],
                [255, 127, 255, 127, 255],
                [127, 255, 0, 255, 127],
                [255, 127, 255, 127, 255],
                [0, 255, 127, 255, 0],
            ],
            np.uint8,
        )

        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()

        # Convert the OpenCV image to a ROS image message
        ros_image = bridge.cv2_to_imgmsg(gray)

        # Publish the ROS image message
        self.image_pub.publish(ros_image)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    gray_scale = Gray_Scale()
    try:
        while not rospy.is_shutdown():
            gray_scale.main()
    except rospy.ROSInterruptException:
        pass
