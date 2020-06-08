'''
Created on 12. 3. 2018

@author: Hijtec
'''

import numpy as np
import cv2, time
from BP_scripts.functions import Calibrate2Cameras, Release

(imgsize, leftMapX, leftMapY, leftROI, rightMapX, 
rightMapY, rightROI, frame, frame2 )= Calibrate2Cameras(0,1);
cap1 = cv2.VideoCapture(0);
cap2 = cv2.VideoCapture(1);
kernel_erode = np.ones((3,3),np.uint8)
kernel_dilate = np.ones((12,12),np.uint8)

def update(imgL,imgR):
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0;
    cv2.imshow('left', imgL);
    cv2.imshow('left', imgR);
    print ('computing disparity...');
    disparity = ((disp-min_disp)/num_disp);
    erosion = cv2.erode(disparity,kernel_erode,iterations = 1);
    dilatation = cv2.dilate(erosion,kernel_dilate,iterations = 1);
    blur = cv2.GaussianBlur(dilatation,(9,9),0)
    return blur
    time.sleep(0.1);

def feed(cap1,cap2):
    ret1, frame1 = cap1.read();
    ret2, frame2 = cap2.read();
    frame1 = cv2.remap(frame1, leftMapX, leftMapY, cv2.INTER_LINEAR); # undistorts the image for further work with it
    frame2 = cv2.remap(frame2, rightMapX, rightMapY, cv2.INTER_LINEAR ); # undistorts the image for further work with it
    return frame1,frame2

def findContours(frame):
    selected = []
    frame=np.uint8(frame)
    cv2.imshow("frame",frame)
    edges = cv2.Canny(frame, 10, 240)
    cv2.imshow("edges", edges)
    (__,contours,___) = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    for c in contours:
        if cv2.contourArea() > 200:
            continue
        selected.append(c);
        
    cv2.drawContours(frame,selected,-1,[0,255,0],2)
    return frame
    
if __name__ == "__main__":
    window_size = 5
    min_disp = 16
    num_disp = 192-min_disp
    blockSize = window_size
    uniquenessRatio = 20
    speckleRange = 10
    speckleWindowSize = 4
    disp12MaxDiff = 60
    P1 = 600
    P2 = 2400
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = window_size,
        uniquenessRatio = uniquenessRatio,
        speckleRange = speckleRange,
        speckleWindowSize = speckleWindowSize,
        disp12MaxDiff = disp12MaxDiff,
        P1 = P1,
        P2 = P2
        )

while True:
    imgL,imgR = feed(cap1,cap2);
    blur = update(imgL, imgR);
    cont = findContours(blur);
    cv2.imshow("contours", cont);
    cv2.imshow('disparity', blur);
    if cv2.waitKey(2) & 0xFF == ord("q"):
        break
Release()
