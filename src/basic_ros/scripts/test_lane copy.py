import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import math
from geometry_msgs.msg import Twist
import threading


class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        # ROS 노드 초기화
        rospy.init_node("image_processor_node", anonymous=True)

        # 카메라 이미지 토픽을 구독하고, 콜백 함수인 image_callback을 등록
        rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.image_callback)

        # 속도와 방향을 제어하는 퍼블리셔 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 이미지의 너비와 높이를 저장하는 변수
        self.img_x = 640
        self.img_y = 480

        # 변수 초기화
        self.previous_point = 0
        self.angle_resolution = math.pi / 640  # 이미지 가로 / 180도
        self.middle_point = 0
        self.cmd_msg = Twist()
        self.angle_speed = 0
        self.speed = 0
        self.bridge = CvBridge()
        self.trackbar_flag = False
        self.create_mask_flag = True
        self.Condition = 0

    def image_binary(self, image):
        # binary 이미지로 변환
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        binary_image = np.zeros_like(gray_image)
        binary_image[gray_image != 0] = 1
        return binary_image

    def direction(self):
        # 주행 방향 결정
        if self.Condition == 0:  # 좌측 차선 주행
            if self.middle_point < 13:
                condition = "left"
            else:
                condition = "right"

        else:  # 우측 차선 주행
            if self.middle_point > 200:
                condition = "left"
            else:
                condition = "right"
        return condition

    def control(self, condition):
        # 주행 제어 설정
        if self.Condition == 0:  # 좌측 차선 주행
            if condition == "left":
                self.angle_speed = abs(self.angle_resolution * abs(self.middle_point - 10))
            elif condition == "right":
                self.angle_speed = -self.angle_resolution * abs(self.middle_point - 80)

        else:  # 우측 차선 주행
            if condition == "left":
                self.angle_speed = -abs(self.angle_resolution * abs(self.middle_point - 320))
            elif condition == "right":
                self.angle_speed = self.angle_resolution * abs(self.middle_point - 240)

    def create_trackbar_init(self):
        def hsv_track(value):
            # 트랙바 값 얻어오기
            self.L_H_Value = cv2.getTrackbarPos("LH", self.win_name)
            self.L_S_Value = cv2.getTrackbarPos("LS", self.win_name)
            self.L_V_Value = cv2.getTrackbarPos("LV", self.win_name)
            self.U_H_Value = cv2.getTrackbarPos("UH", self.win_name)
            self.U_S_Value = cv2.getTrackbarPos("US", self.win_name)
            self.U_V_Value = cv2.getTrackbarPos("UV", self.win_name)

        def left_or_right(value):
            self.Con = cv2.getTrackbarPos("L_or_R", self.win_name)
            self.Condition = self.Con

        self.trackbar_flag = True
        # 트랙바를 위한 윈도우 생성
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 700, 700)
        # HSV 범위에 대한 트랙바 생성
        cv2.createTrackbar("LH", self.win_name, 0, 179, hsv_track)
        cv2.createTrackbar("LS", self.win_name, 0, 255, hsv_track)
        cv2.createTrackbar("LV", self.win_name, 0, 255, hsv_track)
        cv2.createTrackbar("UH", self.win_name, 179, 179, hsv_track)
        cv2.createTrackbar("US", self.win_name, 255, 255, hsv_track)
        cv2.createTrackbar("UV", self.win_name, 255, 255, hsv_track)
        cv2.createTrackbar("L_or_R", self.win_name, 0, 1, left_or_right)

    def create_mask(self):
        self.win_name = "color_detect_hsv"

        if self.trackbar_flag:
            lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
            upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])
            # HSV 범위로 마스크 생성
            self.mask = cv2.inRange(self.cvt_hsv, lower, upper)
            # 마스크를 원본 이미지에 적용하여 최종 결과 얻기
            self.rst = cv2.bitwise_and(self.image, self.image, mask=self.mask)
            lower_string = "lower H,S,V : " + ",".join(str(e) for e in lower.tolist())
            upper_string = "upper H,S,V : " + ",".join(str(e) for e in upper.tolist())

            # 이미지에 텍스트 추가
            cv2.putText(
                self.rst,
                lower_string,
                (25, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                self.rst,
                upper_string,
                (25, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            if self.create_mask_flag:
                cv2.imshow(self.win_name, self.rst)
                key = cv2.waitKey(1)  # 키 입력 대기
                if key == 27:  # ESC 키를 누르면 종료
                    cv2.destroyAllWindows()
                    self.create_mask_flag = False

            # 마스크 값 반환
            return lower, upper

        else:  # 트랙바 윈도우 초기화
            self.create_trackbar_init()

    def apply_mask(self, image, mask):
        # 마스크 적용
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, mask[0], mask[1])
        color = cv2.bitwise_and(image, image, mask=mask)
        return color

    def image_callback(self, msg):
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)  # 카메라 데이터 압축
        self.cvt_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = self.create_mask()  # 사용자 지정 마스크 생성
        camera_data = self.image  # 카메라 데이터 저장
        if self.create_mask_flag == False:
            image_on_mask = self.apply_mask(camera_data, mask)  # 마스크 적용

            if self.Condition == 0:  # 좌측 차선 주행
                print("You are on Left!")
                crop_image = image_on_mask[290:480, 0:380]  # 이미지 자르기
            else:  # 우측 차선 주행
                print("You are on Right!")
                crop_image = image_on_mask[290:480, 320:]
            binary_line = self.image_binary(crop_image)  # binary이미지 변환
            histogram = np.sum(binary_line[170:190, :], axis=0)  # 이미지 세로 합 구하기
            threshold = 10
            indices = np.where(histogram > threshold)[0]  # threshold보다 값이 큰 histogram 리스트 추가
            try:
                self.middle_point = int((min(indices) + max(indices)) / 2)  # min 과 max의 절반 지점을 middle_point로 설정
                self.previous_point = self.middle_point  # 주행 중 차선이 사라질 것을 대비해 중앙점 저장
            except:
                self.middle_point = self.previous_point  # 차선 인식 실패시 이전에 저장한 포인트로 주행
            cd = self.direction()  # 회전방향 결정
            self.control(cd)  # 회전각도 제어
            self.cmd_msg.linear.x = 0.3  # LIMO 주행 속도
            self.cmd_msg.angular.z = self.angle_speed  # LIMO 각속도
            self.pub.publish(self.cmd_msg)  # 속도 publish
            try:
                cv2.rectangle(crop_image, (min(indices), 170), (max(indices), 190), (0, 255, 0), 3)  # 시각화를 위한 화면에 사각형 그리기
            except:
                pass
            cv2.imshow("Hough Lines", crop_image)  # 처리된 이미지 showing
            cv2.imshow("image", self.image)  # 원본 이미지 showing
            cv2.waitKey(1)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()


if __name__ == "__main__":
    image_processor = ImageProcessor()
    image_processor.run()
