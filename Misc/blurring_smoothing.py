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
    lower_red = np.array([120,100,50]);
    upper_red = np.array([255,225,225]);
    
    mask = cv2.inRange(hsv, lower_red, upper_red);
    res = cv2.bitwise_and(frame, frame, mask = mask);
    
    kernel = np.ones((8,8), np.float32)/64; #average from 15x15 pixels
    smoothed = cv2.filter2D(res, -1, kernel); #gives us blur and eliminates background noise by simple math
    
    blur = cv2.GaussianBlur(res, (15,15), 0);
    medianblur = cv2.medianBlur(res, 15);
    bilateralblur = cv2.bilateralFilter(res, 8, 75,75);
    
    
    cv2.imshow("frame",frame)
    cv2.imshow("mask",mask)
    cv2.imshow("res",res)
    cv2.imshow("smoothed",smoothed)
    cv2.imshow("blur",blur)
    cv2.imshow("medianblur",medianblur)
    cv2.imshow("bilateralblur",bilateralblur)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cv2.destroyAllWindows();
cap.release();