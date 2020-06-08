'''
Created on 25. 10. 2017
Zkalibruje vstup kamery, vypíše procentuální chybu kalibrace, ukáže nezkreslený obraz


@author: Hijtec
'''
import cv2
import numpy as np
from BP_scripts.functions import *



cap, mtx, dist, rvecs, tvecs, objpoints, imgpoints = CalibrateCamera(1) # calibrates camera 0 and returns its feed, matrix and distortions and other things, see functions for more info
tot_error = 0;
while True:
    frame = Undistort(cap, mtx, dist); # undistorts the image for further work with it
    cv2.imshow("undistorted{a}".format(a=s), frame);
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error

    print ("total error: ", tot_error/len(objpoints)) #procentuální chyba kalibrace naší kamery
Release()