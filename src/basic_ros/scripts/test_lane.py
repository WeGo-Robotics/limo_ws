import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import math
from geometry_msgs.msg import Twist


class ImageProcessor:
    def __init__(self):
        # CvBridge 인스턴스를 초기화하여 ROS 이미지를 OpenCV 이미지로 변환합니다.
        self.bridge = CvBridge()

        # ROS 노드를 초기화하고 노드 이름을 'image_processor_node'로 설정합니다.
        rospy.init_node("image_processor_node", anonymous=True)

        # 'camera/rgb/image_raw/compressed' 토픽에서 CompressedImage 메시지를 Subscriber하고,
        # 새 메시지가 수신되면 self.image_callback 메서드를 호출합니다.
        # rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.image_callback)

        # 'cmd_vel' 토픽에 Twist 메시지를 발행할 퍼블리셔를 초기화합니다.
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 이미지의 너비와 높이를 저장할 변수를 초기화합니다.
        self.img_x = 640
        self.img_y = 480

        # 노란색을 감지하기 위한 HSV 범위를 설정합니다.
        self.yellow_lower = np.array([0, 50, 50])
        self.yellow_upper = np.array([35, 255, 255])

        # 이전 점을 저장하기 위한 변수를 초기화합니다.
        self.previous_point = 0

        # 각도 해상도 및 중간점을 계산하기 위한 변수를 초기화합니다.
        self.angle_resolution = math.pi / 640  # 이미지 가로 / 180도
        self.middle_point = 0

        # 제어를 위한 Twist 메시지와 각도 속도, 선속도를 저장할 변수들을 초기화합니다.
        self.cmd_msg = Twist()
        self.angle_speed = 0
        self.speed = 0
        # Create the trackbars for the HSV ranges
        self.win_name = "image"
        self.create_trackbar_init()
        # self.hsv_track()

    def create_trackbar_init(self):
        #  Create a named window for the trackbar GUI
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        # cv2.create
        # Create trackbars for the HSV range
        cv2.createTrackbar("LH", self.win_name, 0, 179, self.hsv_track)
        cv2.createTrackbar("LS", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("LV", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("UH", self.win_name, 179, 179, self.hsv_track)
        cv2.createTrackbar("US", self.win_name, 255, 255, self.hsv_track)
        cv2.createTrackbar("UV", self.win_name, 255, 255, self.hsv_track)

    def hsv_track(self, value):
        # Get the values of the trackbars
        self.L_H_Value = cv2.getTrackbarPos("LH", self.win_name)
        self.L_S_Value = cv2.getTrackbarPos("LS", self.win_name)
        self.L_V_Value = cv2.getTrackbarPos("LV", self.win_name)
        self.U_H_Value = cv2.getTrackbarPos("UH", self.win_name)
        self.U_S_Value = cv2.getTrackbarPos("US", self.win_name)
        self.U_V_Value = cv2.getTrackbarPos("UV", self.win_name)

    def detect_color(self, img):
        # 이미지의 평균을 계산하여 mean 변수에 저장합니다.
        # mean = img.mean()

        # # # 화면을 지우고 평균을 출력합니다.
        # # os.system("clear")
        # # print(mean)

        # # 평균에 따라 최소 채도와 밝기를 설정합니다.
        # if mean < 110:
        #     LS = mean - 10
        # else:
        #     LS = 20
        # LV = mean - 10

        # BGR 이미지를 HSV 이미지로 변환합니다.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 최소 채도와 밝기를 이용하여 HSV 범위를 설정합니다.
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])

        # HSV 이미지를 사용하여 노란색만을 감지하는 마스크를 생성합니다.
        mask = cv2.inRange(hsv, lower, upper)

        # 마스크를 이용하여 노란색 부분만 남기고 나머지는 검은색으로 만듭니다.
        color = cv2.bitwise_and(img, img, mask=mask)

        return color

    def img_binary(self, blend_line):
        # blend_line 이미지를 그레이스케일로 변환합니다.
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)

        # 이진 이미지를 생성하고, bin에서 0이 아닌 값은 1로 설정합니다.
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1

        return binary_line

    def direction(self):
        # 중간점을 기준으로 왼쪽 또는 오른쪽으로 회전해야 하는지 결정합니다.
        if self.middle_point < 13:
            condition = "left"
        else:
            condition = "right"

        return condition

    def control(self, condition):
        # 주어진 회전 방향에 따라 각도 속도를 계산합니다.
        if condition == "left":
            self.angle_speed = abs(self.angle_resolution * abs(self.middle_point - 10))
        elif condition == "right":
            self.angle_speed = -self.angle_resolution * abs(self.middle_point - 80)

    def image_callback(self, msg):
        # CompressedImage 메시지를 OpenCV 이미지로 변환합니다.
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # 이미지를 잘라내어 필요한 부분만 가져옵니다.
        crop_image = image[290:480, 0:380]

        # 노란색을 감지하는 단순화된 이미지를 생성합니다.
        simplify_image = self.detect_color(crop_image)

        # 이미지를 이진화하여 선만 남기는 이진 이미지를 생성합니다.
        binary_line = self.img_binary(simplify_image)

        # 이미지의 특정 영역에서 히스토그램을 생성합니다.
        histogram = np.sum(binary_line[170:190, :], axis=0)

        # 히스토그램을 기반으로 중간점을 계산합니다.
        threshold = 10
        indices = np.where(histogram > threshold)[0]
        try:
            self.middle_point = int((min(indices) + max(indices)) / 2)
            self.previous_point = self.middle_point
        except:
            self.middle_point = self.previous_point

        # 회전 방향을 결정하고 각도 속도를 계산합니다.
        cd = self.direction()
        self.control(cd)

        # 이동 및 회전 제어를 위한 Twist 메시지 설정합니다.
        self.cmd_msg.linear.x = 0.3
        self.cmd_msg.angular.z = self.angle_speed

        # Twist 메시지를 발행합니다.
        self.pub.publish(self.cmd_msg)

        try:
            # 중간점을 사각형으로 표시하여 시각화합니다.
            cv2.rectangle(simplify_image, (min(indices), 170), (max(indices), 190), (0, 255, 0), 3)
        except:
            pass

        # 이미지를 표시합니다.
        cv2.imshow("Hough Lines", simplify_image)
        # cv2.imshow(self.win_name, image)
        cv2.waitKey(1)

    def run(self):
        try:
            # ROS 노드를 실행합니다.
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()


if __name__ == "__main__":
    # ImageProcessor 인스턴스를 생성하고 run 메서드를 호출하여 프로그램을 실행합니다.
    image_processor = ImageProcessor()
    image_processor.run()
