import cv2
import numpy as np
blank = np.zeros((480,640,3),np.uint8)
blank[230:250,320:340]=(0,255,0)
cv2.imshow("blank",blank)
cv2.waitKey(0)