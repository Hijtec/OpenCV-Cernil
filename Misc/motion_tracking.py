'''
Created on 24. 10. 2017

@author: Hijtec
'''
import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0);
oldtime = float(0);

while (True):
    ret, frame = cap.read();
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    gray = cv2.GaussianBlur(gray, (21, 21), 0);
    
    try:
        firstFrame
    except(NameError):
        firstFrame = gray;     
        
    frameDelta = cv2.absdiff(firstFrame, gray);
    thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1];
    thresh = cv2.dilate(thresh, None, iterations=2);
    (__,cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < 100:
            continue
 
        # compute the bounding box for the contour, draw it on the frame,
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    if oldtime + 0.1 < time.time():
        oldtime = time.time()
        firstFrame = gray;
        
        
        
    cv2.imshow("tracking", frame);
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();