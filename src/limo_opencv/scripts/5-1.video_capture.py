#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Define a class to capture and publish images from webcam
class Webcam:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("webcam_node", anonymous=True)

        # Create a publisher to publish image message
        self.image_pub = rospy.Publisher("webcam_img", Image, queue_size=1)

        # Create a VideoCapture object to access the default camera (index 0)
        self.capture = cv2.VideoCapture(0)

        # Set the width of the frames to 640 pixels
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)

        # Set the height of the frames to 480 pixels
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Set the frame rate to 30 frames per second
        self.capture.set(cv2.CAP_PROP_FPS, 30)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Set the publishing rate to 10
        # self.rate = rospy.Rate(10)

    # Define the main function of the class
    def main(self):
        # Capture a frame from the camera
        _, img = self.capture.read()

        #  Create a named window for the image
        cv2.namedWindow("video", cv2.WINDOW_NORMAL)

        # Display the image in a window
        cv2.imshow("video", img)

        # Wait for a key press
        cv2.waitKey(1)

        # Convert the OpenCV image to a ROS image message
        webcam_img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

        # Publish the ROS image message
        self.image_pub.publish(webcam_img_msg)

        # Sleep for a certain duration to maintain the publishing rate
        # self.rate.sleep()


if __name__ == "__main__":
    webcam = Webcam()
    try:
        while not rospy.is_shutdown():
            webcam.main()
    except rospy.ROSInterruptException:
        pass
