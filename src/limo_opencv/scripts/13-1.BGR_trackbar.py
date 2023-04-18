#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class BGR_trackbar:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)

        # Set the publishing rate to 10
        # self.rate = rospy.Rate(10)

        # Get the file path of the "limo_opencv" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("limo_opencv")
        self.file_path += "/scripts/"

        # Set the file path for the image files
        self.bgr_path = self.file_path + "HSV.png"

        # The name of the window for the trackbar GUI
        self.win_name = "color_detect_bgr"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Create the trackbars for the BGR ranges
        self.create_trackbar_init()

    def create_trackbar_init(self):
        #  Create a named window for the trackbar GUI
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)

        # Create trackbars for the BGR range
        cv2.createTrackbar("LB", self.win_name, 0, 255, self.bgr_track)
        cv2.createTrackbar("LG", self.win_name, 0, 255, self.bgr_track)
        cv2.createTrackbar("LR", self.win_name, 0, 255, self.bgr_track)
        cv2.createTrackbar("UB", self.win_name, 255, 255, self.bgr_track)
        cv2.createTrackbar("UG", self.win_name, 255, 255, self.bgr_track)
        cv2.createTrackbar("UR", self.win_name, 255, 255, self.bgr_track)

    def bgr_track(self, value):
        # Get the values of the trackbars
        self.L_B_Value = cv2.getTrackbarPos("LB", self.win_name)
        self.L_G_Value = cv2.getTrackbarPos("LG", self.win_name)
        self.L_R_Value = cv2.getTrackbarPos("LR", self.win_name)
        self.U_B_Value = cv2.getTrackbarPos("UB", self.win_name)
        self.U_G_Value = cv2.getTrackbarPos("UG", self.win_name)
        self.U_R_Value = cv2.getTrackbarPos("UR", self.win_name)

    # Define the main function of the class
    def main(self):
        # Read the image in BGR format using OpenCV
        color_img = cv2.imread(self.bgr_path, cv2.IMREAD_COLOR)

        # Define the BGR range using the trackbar values
        lower = np.array([self.L_B_Value, self.L_G_Value, self.L_R_Value])
        upper = np.array([self.U_B_Value, self.U_G_Value, self.U_R_Value])

        # Create a mask using the RGB range
        mask = cv2.inRange(color_img, lower, upper)

        # Apply the mask to the original image to obtain the final result
        rst = cv2.bitwise_and(color_img, color_img, mask=mask)

        # Convert np array to string for display purposes
        lower_string = "lower B,G,R : " + ",".join(str(e) for e in lower.tolist())
        upper_string = "upper B,G,R : " + ",".join(str(e) for e in upper.tolist())

        # Add text to the image
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

        # Display the image in a window
        cv2.imshow(self.win_name, rst)

        # Wait for a key press
        cv2.waitKey(1)

        # Convert the OpenCV image to a ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(rst, "bgr8")

        # Publish the ROS image message
        self.image_cvt_pub.publish(image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        # self.rate.sleep()


if __name__ == "__main__":
    bgr_trackbar = BGR_trackbar()
    try:
        while not rospy.is_shutdown():
            bgr_trackbar.main()
    except rospy.ROSInterruptException:
        pass
