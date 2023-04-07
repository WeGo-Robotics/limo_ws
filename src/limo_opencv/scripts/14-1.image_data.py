#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os


# Define a class called Image_data
class Image_data:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_data_node", anonymous=False)

        # Create publishers to publish the images
        self.image_pub_color = rospy.Publisher("bgr_img", Image, queue_size=10)

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
        col, row, channel = img.shape
        os.system("clear")
        print(f"--------------------")
        for i in range(0, col):
            for j in range(0, row):
                print(
                    f"img.item({i,j}) : {img.item(i,j,0),img.item(i,j,1),img.item(i,j,2)}"
                )

        print(f"img.shape : {img.shape}")
        print(f"img.size : {img.size}")
        print(f"img.dtype : {img.dtype}")

        # set to white (0, 0)
        img.itemset((0, 0, 0), 255)
        img.itemset((0, 0, 1), 255)
        img.itemset((0, 0, 2), 255)

        # set to white-gray (5, 0)
        img.itemset((0, 5, 0), 192)
        img.itemset((0, 5, 1), 192)
        img.itemset((0, 5, 2), 192)

        # set to dark-gray (5, 0)
        img.itemset((5, 0, 0), 96)
        img.itemset((5, 0, 1), 96)
        img.itemset((5, 0, 2), 96)

        # set to black (5, 5)
        img.itemset((5, 5, 0), 0)
        img.itemset((5, 5, 1), 0)
        img.itemset((5, 5, 2), 0)
        print(f"--------------------")

        # Convert the OpenCV images to ROS image messages
        ros_image_color = self.bridge.cv2_to_imgmsg(img, "bgr8")

        # Publish the ROS image messages
        self.image_pub_color.publish(ros_image_color)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    image_data = Image_data()
    try:
        while not rospy.is_shutdown():
            image_data.main()
    except rospy.ROSInterruptException:
        pass
