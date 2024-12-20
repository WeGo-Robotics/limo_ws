import cv2
import numpy as np
blank = np.zeros((480,640,3),np.uint8)
cv2.circle(blank,(320,240),20,(0,255,255),-1)
cv2.rectangle(blank,(100,200),(200,400),(0,255,0),10)
cv2.line(blank,(400,200),(500,300),(255,0,0),5)
pt1=(100,100)
pt2=(100,300)
pt3=(200,400)
pt4=(400,200)
pt5=(300,100)
pts = np.array((pt1,pt2,pt3,pt4,pt5))
cv2.polylines(blank,[pts],True,(0,0,255),5)
cv2.fillPoly(blank,[pts],(0,0,255))
cv2.imshow("blank",blank)
cv2.putText(blank,"wego",(100,200),cv2.FONT_HERSHEY_COMPLEX,3,(255,255,255),5)
cv2.waitKey(10000)