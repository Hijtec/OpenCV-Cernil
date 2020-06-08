'''
Created on 10. 11. 2017

@author: Hijtec
'''
import cv2
import numpy as np
from BP_scripts.functions import Undistort, Calibrate2Cameras, Release

cap,cap2,mtx,mtx2,dist,dist2 = Calibrate2Cameras(1,2)# calibrates camera 0 and returns its feed, matrix and distortions and other things, see functions for more info

while True:
    frame = Undistort(cap,mtx,dist); # undistorts the image for further work with it
    frame2 = Undistort(cap2,mtx2,dist2); # undistorts the image for further work with it
    cv2.imshow("undistorted1", frame);
    cv2.imshow("undistorted2", frame2);
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
Release()