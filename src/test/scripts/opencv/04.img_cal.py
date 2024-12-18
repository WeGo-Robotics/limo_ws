import cv2
import numpy as np

img = cv2.imread('img.jpg',cv2.IMREAD_COLOR)
blank = np.zeros_like(img,np.uint8)
blank_inv = blank+255
cv2.putText(blank,"wego",(80,260),cv2.FONT_HERSHEY_COMPLEX,6,(255,255,255),15)
cv2.putText(blank_inv,"wego",(80,260),cv2.FONT_HERSHEY_COMPLEX,6,(0,0,0),15)
add_img1 = cv2.add(img,blank)
add_img2 = cv2.add(img,blank_inv)
subtract_img1 = cv2.subtract(img,blank)
subtract_img2 = cv2.subtract(img,blank_inv)
alpha = 0.8
add_weight_img = cv2.addWeighted(img,alpha,blank,1-alpha,1)

cv2.namedWindow("yosemite",cv2.WINDOW_NORMAL)
cv2.imshow("yosemite",img)
cv2.imshow("blank",blank)
cv2.imshow("add_weight_img",add_weight_img)
cv2.imshow("add_img1",add_img1)
cv2.imshow("add_img2",add_img2)
cv2.imshow("subtract_img1",subtract_img1)
cv2.imshow("subtract_img2",subtract_img2)
cv2.imshow("blank_inv",blank_inv)
cv2.waitKey(0)