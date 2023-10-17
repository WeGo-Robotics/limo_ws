import cv2
import numpy as np


blank = np.zeros((500, 500, 3), np.uint8)
width, height, shape = blank.shape

# cv2.putText(blank, "<- RIGHT", (width // 4, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
# cv2.putText(blank, "LEFT ->", (width // 4 * 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
# cv2.putText(blank, "STRAIGHT", (width // 5 * 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2, cv2.LINE_AA)
cv2.putText(blank, "STOP", (width // 5 * 2, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 2, [0, 0, 255], 2, cv2.LINE_AA)

while True:
    cv2.namedWindow("blank", cv2.WINDOW_NORMAL)
    cv2.imshow("blank", blank)
    cv2.waitKey(1)
