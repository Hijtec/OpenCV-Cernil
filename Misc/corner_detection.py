'''
Created on 8. 10. 2017

Detects corners and shows them in an image
@author: Hijtec
'''

import cv2
import numpy as np
import time

img = cv2.imread("corner_detection_sample.jpg");
cap = cv2.VideoCapture(1);
while True:
    ret, frame = cap.read();
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    

    corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 100) #where, how many, quality, minimum distance between them
    corners = np.int0(corners); #datatype conversion

    for corner in corners:
        x, y = corner.ravel();
        cv2.circle(frame, (x, y),  5,(255,0,255), -1);

    cv2.imshow("Corners", frame);  
    
    time.sleep(2)