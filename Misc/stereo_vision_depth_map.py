'''
Created on 21. 10. 2017

@author: Hijtec
'''

import numpy as np
import cv2
from matplotlib import pyplot as plt


cap1 = cv2.VideoCapture(0);
cap2 = cv2.VideoCapture(1);

def feed_gray(cap):
        __, frame = cap.read();
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);
        return gray

window_size = 5
min_disp = 32
num_disp = 112-min_disp
window_size = 3
min_disp = 16
num_disp = 112-min_disp
stereo = cv2.StereoBM_create(16, 5);
stereo = cv2.StereoSGBM_create(minDisparity = 16,
                               numDisparities = 3,
                               blockSize = 3,
                               P1 = 0,
                               P2 = 0,
                               disp12MaxDiff = 0,
                               preFilterCap = 0,
                               uniquenessRatio = 0,
                               speckleWindowSize = 0,
                               speckleRange = 0,
                               );


while True:
    gray1 = feed_gray(cap1);
    gray2 = feed_gray(cap2);
    
    disparity = stereo.compute(gray1, gray2).astype(np.float32)/16.0
    disparity = (disparity-min_disp)/num_disp
    cv2.imshow("disparity", disparity)
    #plt.imshow(disparity,'gray')
    #plt.show()
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
cap1.release()
cap2.release()
cv2.destroyAllWindows