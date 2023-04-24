#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class Img_Data_Control:
    def __init__(self):
        rospy.init_node("camera_node")
        rospy.Subscriber(
            "/camera/rgb/image_raw/compressed", CompressedImage, self.camera_CB
        )
        self.pub_roi_img = rospy.Publisher(
            "/camera/roi/compressed", CompressedImage, queue_size=10
        )
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.bridge = CvBridge()
        self.basic_speed = 0.3

    def color_case(self, avg_h, avg_s, avg_v):
        white = [255, 255, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]
        cyan = [255, 255, 0]
        magenta = [255, 0, 255]
        yellow = [0, 255, 255]

        if avg_s > 64:
            if avg_h < 15 or 165 <= avg_h:
                color_name = "red"
                color = red
                action_name = "back"
                action = -self.basic_speed
            elif 15 <= avg_h < 45:
                color_name = "yellow"
                color = yellow
                action_name = "slow"
                action = self.basic_speed / 2
            elif 45 <= avg_h < 75:
                color_name = "green"
                color = green
                action_name = "go"
                action = self.basic_speed

            elif 75 <= avg_h < 105:
                color_name = "cyan"
                color = cyan
                action_name = "fast"
                action = self.basic_speed * 2
            elif 105 <= avg_h < 135:
                color_name = "blue"
                color = blue
                action_name = "right"
                action = self.basic_speed
            elif 135 <= avg_h < 165:
                color_name = "magenta"
                color = magenta
                action_name = "left"
                action = -self.basic_speed
            h_value = round(avg_h)

        else:
            color = white
            color_name = ""
            h_value = ""
            action_name = "stop"
            action = 0
        return color, color_name, h_value, action_name, action

    def draw_img(
        self, origin_img, roi_x, roi_y, color, color_name, h_value, action_name, action
    ):
        roi_img = cv2.rectangle(
            origin_img, [roi_x[0], roi_y[0]], [roi_x[1], roi_y[1]], color, 5
        )
        if color_name:
            roi_img = cv2.putText(
                roi_img,
                "color : " + color_name,
                [roi_x[0] + 20, roi_y[0] + 40],
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                color,
                2,
            )
            roi_img = cv2.putText(
                roi_img,
                "h_value : " + str(h_value),
                [roi_x[0] + 20, (roi_y[0] + roi_y[1]) // 2 - 20],
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                color,
                2,
            )

            roi_img = cv2.putText(
                roi_img,
                "action : " + action_name,
                [roi_x[0] + 20, (roi_y[0] + roi_y[1]) // 2 + 40],
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                color,
                2,
            )
            roi_img = cv2.putText(
                roi_img,
                "value : " + str(action),
                [roi_x[0] + 20, roi_y[1] - 20],
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                color,
                2,
            )

        else:
            roi_img = cv2.putText(
                roi_img,
                "detect color zone",
                [roi_x[0] + 5, (roi_y[0] + roi_y[1]) // 2 + 5],
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                color,
                2,
            )
        return roi_img

    def camera_CB(self, data):
        twist_msg = Twist()
        origin_img = self.bridge.compressed_imgmsg_to_cv2(data)

        img_x, img_y = origin_img.shape[1], origin_img.shape[0]
        roi_x = [img_x // 4, img_x // 4 * 3]
        roi_y = [img_y // 4, img_y // 4 * 3]
        roi_img = origin_img[roi_y[0] : roi_y[1], roi_x[0] : roi_x[1]]

        hsv_roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_roi_img)
        avg_h = np.average(h)
        avg_s = np.average(s)
        avg_v = np.average(v)

        color, color_name, h_value, action_name, action = self.color_case(
            avg_h, avg_s, avg_v
        )
        roi_img = self.draw_img(
            origin_img, roi_x, roi_y, color, color_name, h_value, action_name, action
        )

        if action_name == "left" or action_name == "right":
            twist_msg.angular.z = action
        else:
            twist_msg.linear.x = action
        roi_img_msg = self.bridge.cv2_to_compressed_imgmsg(roi_img)
        self.pub_roi_img.publish(roi_img_msg)
        self.pub_cmd_vel.publish(twist_msg)


if __name__ == "__main__":
    img_data_control = Img_Data_Control()
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
