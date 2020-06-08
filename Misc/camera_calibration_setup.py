'''
Created on 20. 10. 2017

@author: Hijtec
'''
import numpy as np
import cv2
import glob


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp1 = np.zeros((6*9,3), np.float32)
objp1[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp2 = np.zeros((6*9,3), np.float32)
objp1[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints1 = [] # 3d point in real world space
imgpoints1 = [] # 2d points in image plane.
objpoints2 = [] # 3d point in real world space
imgpoints2 = [] # 2d points in image plane.

cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

while True:
    __, frame1 = cap1.read();
    gray1 = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    __, frame2 = cap2.read();
    gray2 = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret1, corners1 = cv2.findChessboardCorners(gray1, (9,6),None)
    ret2, corners2 = cv2.findChessboardCorners(gray2, (9,6),None)

    # If found, add object points, image points (after refining them)
    
    objpoints2.append(objp2)
    if ret1 == True:
        objpoints1.append(objp1)
        corners1_2 = cv2.cornerSubPix(gray1,corners1,(11,11),(-1,-1),criteria)
        imgpoints1.append(corners1_2)
        # Draw and display the corners
        frame1 = cv2.drawChessboardCorners(frame1, (9,6), corners1_2,ret1)
        
    if ret2 == True:
        objpoints2.append(objp2)
        corners2_2 = cv2.cornerSubPix(gray2,corners2,(11,11),(-1,-1),criteria)
        imgpoints2.append(corners1_2)
        # Draw and display the corners
        frame2 = cv2.drawChessboardCorners(frame2, (9,6), corners2_2,ret2)    
            
    cv2.imshow("cap1",frame1)
    cv2.imshow("cap2",frame2)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    
cap1.release()
cap2.release()
cv2.destroyAllWindows()