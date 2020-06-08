'''
Created on 19. 11. 2017

@author: Hijtec
'''
import cv2
from BP_scripts.functions import Calibrate2Cameras
imgsize, leftMapX, leftMapY, leftROI, rightMapX, rightMapY, rightROI, frame, frame2 = Calibrate2Cameras(0,1);

dstL = cv2.remap(frame, leftMapX, leftMapY,cv2.INTER_LINEAR)
dstR = cv2.remap(frame2, rightMapX, rightMapY,cv2.INTER_LINEAR)

cv2.imshow("left",dstL)
cv2.imshow("right",dstR)

file = "stereo/capone{a}.png".format(a=1);
cv2.imwrite(file, dstL);
        
file = "stereo/captwo{a}.png".format(a=1);
cv2.imwrite(file, dstR);
cv2.waitKey()