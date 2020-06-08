'''
Created on 7. 10. 2017

@author: Hijtec
'''
import cv2
import numpy as np

cap = cv2.VideoCapture(0);

while (True):
    ret, frame = cap.read();
    
    laplacian = cv2.Laplacian(frame, cv2.CV_64F);
    sobelx = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=5);#gradient in x direction
    sobely = cv2.Sobel(frame, cv2.CV_64F, 0, 1, ksize=5);#gradient in y direction
    
    edges = cv2.Canny(frame, 100, 200)#edge detection , 100,200 give the most convenient results
    
    cv2.imshow("frame",frame);
    cv2.imshow("laplacian",laplacian);
    cv2.imshow("sobelx",sobelx);
    cv2.imshow("sobely",sobely);
    cv2.imshow("edges",edges);
    
    
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
cv2.destroyAllWindows();
cap.release();