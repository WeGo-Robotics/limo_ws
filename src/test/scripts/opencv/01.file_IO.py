import cv2

img = cv2.imread('img.jpg',cv2.IMREAD_COLOR)
cv2.namedWindow("yosemite",cv2.WINDOW_NORMAL)
cv2.imshow("yosemite",img)
cv2.waitKey(0)
# cv2.imwrite("yosemite_gray.jpg",img)