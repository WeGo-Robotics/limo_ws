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

        pts1 = np.array([[300, 150], [400, 150], [350, 250]])
        pts2 = np.array([[300, 250], [300, 325], [375, 325]])
        pts3 = np.array([[425, 200], [500, 150], [500, 250]])

        # Add some colored pixels to the image
        img[175, 125] = white
        img[0:50, 50:100] = blue
        img[50:100, 100:150] = green
        img[100:150, 150:200] = red
        img[200:400, 95:105] = cyan

        # Add some lines, circles, rectangles, and polygons to the image
        cv2.line(img, (100, 200), (300, 400), magenta, 5)
        cv2.line(img, (150, 300), (150, 300), yellow, 10)
        cv2.circle(img, (300, 400), 50, blue, 5)
        cv2.ellipse(img, (250, 200), (50, 25), 45, 0, 360, cyan, 2)
        cv2.rectangle(img, (400, 300), (600, 400), green, 5)
        cv2.polylines(img, [pts1], True, red, 2)
        cv2.polylines(img, [pts2], False, magenta, 2)
        cv2.fillPoly(img, [pts3], yellow)
        cv2.putText(
            img,
            "Hello World",
            (225, 100),
            cv2.FONT_HERSHEY_COMPLEX,
            2,
            white,
            3,
        )

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
