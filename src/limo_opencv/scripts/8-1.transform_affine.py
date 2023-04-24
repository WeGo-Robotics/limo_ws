#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Transform:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.affine_pub = rospy.Publisher("affine_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Define color variables for blue, green, red, yellow, and white
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]
        yellow = [0, 255, 255]
        white = [255, 255, 255]

        # Define the shape of the image as [1000, 1000]
        shape = [1000, 1000]

        # Set the height and width variables based on the shape of the image
        height, width = shape

        # Define an 8x10 2D numpy array representing a chessboard pattern
        chessboard = np.uint8(
            [
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
            ]
        )

        # Resize the chessboard image to the specified shape, binarize it, and stack it with 3 channels
        resize_cb = cv2.resize(chessboard, shape)
        resize_cb[resize_cb <= 127] = 0
        resize_cb[resize_cb > 127] = 255
        resize_cb_3d = np.dstack((resize_cb, resize_cb, resize_cb))

        # Define 3 points on the chessboard image and draw circles on them in blue, green, and red colors
        a_point1 = [100, 100]
        a_point2 = [100, 200]
        a_point3 = [200, 100]

        #  Draw a blue circle with radius 10 pixels on point1
        cv2.circle(resize_cb_3d, a_point1, 10, blue, -1)

        # Draw a green circle with radius 10 pixels on point2
        cv2.circle(resize_cb_3d, a_point2, 10, green, -1)

        # Draw a red circle with radius 10 pixels on point3
        cv2.circle(resize_cb_3d, a_point3, 10, red, -1)

        a_src_point = np.float32(
            [a_point1, a_point2, a_point3],
        )

        a_dst_point = np.float32(
            [
                [a_point1[0], a_point1[1]],
                [a_point2[0] * 2, a_point2[1] * 2],
                [a_point3[0] * 2, a_point3[1] * 2],
            ]
        )

        # Get affine transformation matrix and its inverse
        a_matrix = cv2.getAffineTransform(a_src_point, a_dst_point)
        a_matrix_inv = cv2.getAffineTransform(a_dst_point, a_src_point)

        # Apply affine transformation to image
        a_cb_3d = cv2.warpAffine(resize_cb_3d, a_matrix, (width, height))

        # Concatenate original and transformed chessboard images horizontally with lines to indicate points
        a_dst = cv2.hconcat([resize_cb_3d, a_cb_3d])
        cv2.line(
            a_dst,
            a_point1,
            (a_point1[0] + width, a_point1[1]),
            blue,
            10,
        )
        cv2.line(
            a_dst,
            a_point2,
            (a_point2[0] * 2 + width, a_point2[1] * 2),
            green,
            10,
        )
        cv2.line(
            a_dst,
            a_point3,
            (a_point3[0] * 2 + width, a_point3[1] * 2),
            red,
            10,
        )

        # Convert the OpenCV image to a ROS image message
        a_image_msg = self.bridge.cv2_to_imgmsg(a_dst, "bgr8")

        # Publish the ROS image message
        self.affine_pub.publish(a_image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    transform = Transform()
    try:
        while not rospy.is_shutdown():
            transform.main()
    except rospy.ROSInterruptException:
        pass
