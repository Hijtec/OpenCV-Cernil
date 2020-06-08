import numpy as np
import cv2
from BP_scripts.functions import Release, Calibrate2Cameras, Undistort

imgsize, leftMapX, leftMapY, leftROI, rightMapX, rightMapY, rightROI, frame, frame2 = Calibrate2Cameras(2,1)# calibrates camera 0 and returns its feed, matrix and distortions and other things, see functions for more info
cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)
il=1;
ir=1;
while True:
    ret1, frame1 = cap1.read();
    ret2, frame2 = cap2.read();
    frame1 = cv2.remap(frame1, leftMapX, leftMapY, cv2.INTER_LINEAR); # undistorts the image for further work with it
    frame2 = cv2.remap(frame2, rightMapX, rightMapY, cv2.INTER_LINEAR ); # undistorts the image for further work with it
    cv2.imshow("undistorted1", frame1);
    cv2.imshow("undistorted2", frame2);
    
    if cv2.waitKey(2) & 0xFF == ord("f"):
        file1 = "stereo/capone{a}.png".format(a=il);
        il+=1;
        file2 = "stereo/captwo{a}.png".format(a=ir);
        ir+=1;
        cv2.imwrite(file1, frame1)
        cv2.imwrite(file2, frame2);
    if cv2.waitKey(2) & 0xFF == ord("q"):
        break
Release();