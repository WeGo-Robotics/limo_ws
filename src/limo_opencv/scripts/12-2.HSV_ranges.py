#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSV_In_Range:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image files
        self.hsv_path = self.file_path + "HSV.png"
        self.win_name = "color_detect_hsv"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Read the image in BGR format using OpenCV
        color_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)

        # Convert from color image to grayscale image
        cvt_hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

        # Set the lower and upper bounds for the color range in the HSV color space
        lower = np.array([0, 0, 0])
        upper = np.array([179, 255, 255])

        # Create a mask that extracts the pixels in the specified color range
        mask = cv2.inRange(cvt_hsv, lower, upper)

        # Apply the mask to the original color image to obtain the masked image
        rst = cv2.bitwise_and(color_img, color_img, mask=mask)

        # Convert the lower and upper bounds to strings for display purposes
        lower_string = "lower H,S,V : " + ",".join(str(e) for e in lower.tolist())
        upper_string = "upper H,S,V : " + ",".join(str(e) for e in upper.tolist())

        # Add text to the masked image indicating the lower and upper bounds of the color range
        cv2.putText(
            rst,
            lower_string,
            (25, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
        )
        cv2.putText(
            rst,
            upper_string,
            (25, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
        )

        # Convert the OpenCV image to a ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(rst, "bgr8")

        # Publish the ROS image message
        self.image_cvt_pub.publish(image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    hsv_in_range = HSV_In_Range()
    try:
        while not rospy.is_shutdown():
            hsv_in_range.main()
    except rospy.ROSInterruptException:
        pass
