#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os


# Define a class called Merge
class Merge:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("merge_node", anonymous=False)

        # Create publishers to publish the images
        self.image_pub_color = rospy.Publisher("color_img", Image, queue_size=1)
        self.image_pub_bgr = rospy.Publisher("bgr_img", Image, queue_size=1)
        self.image_pub_brg = rospy.Publisher("brg_img", Image, queue_size=1)
        self.image_pub_gbr = rospy.Publisher("gbr_img", Image, queue_size=1)
        self.image_pub_grb = rospy.Publisher("grb_img", Image, queue_size=1)
        self.image_pub_rbg = rospy.Publisher("rbg_img", Image, queue_size=1)
        self.image_pub_rgb = rospy.Publisher("rgb_img", Image, queue_size=1)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.img_path = self.file_path + "yosemite.jpg"

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
        img_bgr = cv2.merge([b, g, r])
        img_brg = cv2.merge([b, r, g])
        img_gbr = cv2.merge([g, b, r])
        img_grb = cv2.merge([g, r, b])
        img_rbg = cv2.merge([r, b, g])
        img_rgb = cv2.merge([r, g, b])

        # Convert the OpenCV images to ROS image messages
        ros_image_color = self.bridge.cv2_to_imgmsg(img, "bgr8")
        ros_image_bgr = self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
        ros_image_brg = self.bridge.cv2_to_imgmsg(img_brg, "bgr8")
        ros_image_gbr = self.bridge.cv2_to_imgmsg(img_gbr, "bgr8")
        ros_image_grb = self.bridge.cv2_to_imgmsg(img_grb, "bgr8")
        ros_image_rbg = self.bridge.cv2_to_imgmsg(img_rbg, "bgr8")
        ros_image_rgb = self.bridge.cv2_to_imgmsg(img_rgb, "bgr8")

        # Publish the ROS image messages
        self.image_pub_color.publish(ros_image_color)
        self.image_pub_bgr.publish(ros_image_bgr)
        self.image_pub_brg.publish(ros_image_brg)
        self.image_pub_gbr.publish(ros_image_gbr)
        self.image_pub_grb.publish(ros_image_grb)
        self.image_pub_rbg.publish(ros_image_rbg)
        self.image_pub_rgb.publish(ros_image_rgb)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    merge = Merge()
    try:
        while not rospy.is_shutdown():
            merge.main()
    except rospy.ROSInterruptException:
        pass
