import numpy as np
import cv2
from BP_scripts.functions import Release, Calibrate2Cameras, Undistort

cap,cap2,mtx,mtx2,dist,dist2 = Calibrate2Cameras(1,2)# calibrates camera 0 and returns its feed, matrix and distortions and other things, see functions for more info

il=1;
ir=1;
while True:
    frame = Undistort(cap,mtx,dist); # undistorts the image for further work with it
    frame2 = Undistort(cap2,mtx2,dist2); # undistorts the image for further work with it
    cv2.imshow("undistorted1", frame);
    cv2.imshow("undistorted2", frame2);
    
    file = "stereo/capone{a}.png".format(a=il);
    il+=1;
    cv2.imwrite(file, frame);
        
    file = "stereo/captwo{a}.png".format(a=ir);
    ir+=1;
    cv2.imwrite(file, frame2);
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
Release();