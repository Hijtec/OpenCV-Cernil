'''
Created on 9. 10. 2017

Reducing image background by detecting motion
@author: Hijtec
'''
import cv2
import numpy as np

cap = cv2.VideoCapture(0);
fgbg = cv2.createBackgroundSubtractorMOG2();

while(1):
    ret, frame = cap.read();
    
    fgmask = fgbg.apply(frame);
    
    cv2.imshow("fgmask", frame);
    cv2.imshow("frame", fgmask);
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();
