import cv2

img = cv2.imread('img.jpg',cv2.IMREAD_COLOR)
cv2.namedWindow("yosemite",cv2.WINDOW_NORMAL)
print(img[100,200])
print(img.shape)
b,g,r = cv2.split(img)
rgb_img = cv2.merge([g,b,r])
cv2.imshow("yosemite",img)
cv2.imshow("b",b)
cv2.imshow("g",g)
cv2.imshow("r",r)
cv2.imshow("rgb_img",rgb_img)
cv2.waitKey(0)