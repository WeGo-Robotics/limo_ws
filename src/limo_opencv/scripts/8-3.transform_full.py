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

        # Create publishers to publish the images
        self.affine_pub = rospy.Publisher("affine_img", Image, queue_size=10)
        self.perspective_pub = rospy.Publisher("perspective_img", Image, queue_size=10)

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

        # Define a 100x100 2D numpy array with two diagonal white lines
        two_lines = np.zeros((100, 100), np.uint8)
        cv2.line(two_lines, (10, 90), (40, 50), white, 2)
        cv2.line(two_lines, (90, 90), (60, 50), white, 2)

        # Resize the chessboard image to the specified shape, binarize it, and stack it with 3 channels
        resize_cb = cv2.resize(chessboard, shape)
        resize_cb[resize_cb <= 127] = 0
        resize_cb[resize_cb > 127] = 255
        resize_cb_3d = np.dstack((resize_cb, resize_cb, resize_cb))

        # Resize the "two_lines" array to the specified shape, binarize it, and stack it with 3 channels
        resize_tl = cv2.resize(two_lines, shape)
        resize_tl[resize_tl <= 127] = 0
        resize_tl[resize_tl > 127] = 255
        resize_tl_3d = np.dstack((resize_tl, resize_tl, resize_tl))

        # Define 3 points on the chessboard image and draw circles on them in blue, green, and red colors
        a_point1 = [100, 100]
        a_point2 = [100, 200]
        a_point3 = [200, 100]

        # Draw a blue circle with radius 10 pixels on point1
        cv2.circle(resize_cb_3d, a_point1, 10, blue, -1)

        # Draw a green circle with radius 10 pixels on point2
        cv2.circle(resize_cb_3d, a_point2, 10, green, -1)

        # Draw a red circle with radius 10 pixels on point3
        cv2.circle(resize_cb_3d, a_point3, 10, red, -1)

        # Define 4 points on the "two_lines" array and draw circles on them in blue, green, red, and yellow colors
        p_point1 = [100, 900]
        p_point2 = [400, 500]
        p_point3 = [600, 500]
        p_point4 = [900, 900]

        # Draw a blue circle with radius 20 pixels on point1
        cv2.circle(resize_tl_3d, p_point1, 20, blue, -1)
        # Draw a blue circle with radius 20 pixels on point2
        cv2.circle(resize_tl_3d, p_point2, 20, green, -1)
        # Draw a blue circle with radius 20 pixels on point3
        cv2.circle(resize_tl_3d, p_point3, 20, red, -1)
        # Draw a blue circle with radius 20 pixels on point4
        cv2.circle(resize_tl_3d, p_point4, 20, yellow, -1)

        # Define three source points for affine transformation
        a_src_point = np.float32(
            [a_point1, a_point2, a_point3],
        )

        # Define three destination points for affine transformation
        a_dst_point = np.float32(
            [
                [a_point1[0], a_point1[1]],
                [a_point2[0] * 2, a_point2[1] * 2],
                [a_point3[0] * 2, a_point3[1] * 2],
            ]
        )
        # Define four source points for perspective transformation
        p_src_point = np.float32([p_point1, p_point2, p_point3, p_point4])

        # Define three destination points for perspective transformation
        p_dst_point = np.float32([[100, 1000], [100, 0], [900, 0], [900, 1000]])

        # Get affine transformation matrix and its inverse
        a_matrix = cv2.getAffineTransform(a_src_point, a_dst_point)
        a_matrix_inv = cv2.getAffineTransform(a_dst_point, a_src_point)

        # Get perspective transformation matrix and its inverse
        p_matrix = cv2.getPerspectiveTransform(p_src_point, p_dst_point)
        p_matrix_inv = cv2.getPerspectiveTransform(p_dst_point, p_src_point)

        # Apply affine and perspective transformations to images
        a_cb_3d = cv2.warpAffine(resize_cb_3d, a_matrix, (width, height))
        p_tl_3d = cv2.warpPerspective(resize_tl_3d, p_matrix, (width, height))

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

        # Concatenate original and transformed two_lines images horizontally with lines to indicate points
        p_dst = cv2.hconcat([resize_tl_3d, p_tl_3d])
        cv2.line(
            p_dst,
            p_point1,
            ([100 + width, 1000]),
            blue,
            10,
        )
        cv2.line(
            p_dst,
            p_point2,
            ([100 + width, 0]),
            green,
            10,
        )
        cv2.line(
            p_dst,
            p_point3,
            ([900 + width, 0]),
            red,
            10,
        )
        cv2.line(
            p_dst,
            p_point4,
            ([900 + width, 1000]),
            yellow,
            10,
        )

        # Convert the OpenCV image to a ROS image message
        a_image_msg = self.bridge.cv2_to_imgmsg(a_dst, "bgr8")
        p_image_msg = self.bridge.cv2_to_imgmsg(p_dst, "bgr8")

        # Publish the ROS image message
        self.affine_pub.publish(a_image_msg)
        self.perspective_pub.publish(p_image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    transform = Transform()
    try:
        while not rospy.is_shutdown():
            transform.main()
    except rospy.ROSInterruptException:
        pass
