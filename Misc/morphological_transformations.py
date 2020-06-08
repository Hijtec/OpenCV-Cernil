'''
Created on 7. 10. 2017

@author: Hijtec
'''
import cv2
import numpy as np

cap = cv2.VideoCapture(0);

while (True):
    ret, frame = cap.read();
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);
    
    #HSV = Hue, Saturation, Value
    lower_red = np.array([80,100,50]);
    upper_red = np.array([130,255,200]);
    
    mask = cv2.inRange(hsv, lower_red, upper_red);
    res = cv2.bitwise_and(frame, frame, mask = mask);
    
    kernel = np.ones((5,5), np.uint8);
    erosion = cv2.erode(mask, kernel, iterations = 1); #tries to get rid of noise (creates some too tho)
    dilation = cv2.dilate(mask, kernel, iterations = 1); #tries to capture every change of demanded mask in feed
    
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # gets rid of false positives (background noise)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # gets rid of false negatives (noise in the demanded mask)
    
    cv2.imshow("frame",frame);
    cv2.imshow("res",res);
    cv2.imshow("erosion",erosion);
    cv2.imshow("dilation",dilation);
    cv2.imshow("opening",opening);
    cv2.imshow("closing",closing);
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
cv2.destroyAllWindows();
cap.release();