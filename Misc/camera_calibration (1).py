'''
Created on 20. 10. 2017

@author: Hijtec
'''
import numpy as np
import cv2
import glob, time

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

objpoints = []
imgpoints = []

cap = cv2.VideoCapture(0)
i = 0

while i<=20:
    __, frame = cap.read();
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        frame = cv2.drawChessboardCorners(frame, (9,6), corners2,ret)
        i +=1
        time.sleep(0.5)
        
    
    cv2.imshow("cap",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
#print(ret,mtx,dist,rvecs,tvecs);

#h, w = frame.shape[:2];
#newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h));

#undst = cv2.undistort(frame,mtx,dist, None, newcameramtx);
#x,y,w,h = roi;
#undst = undst[y:y+h, x:x+w];

while True:
    ___,frame = cap.read()
    h, w = frame.shape[:2];
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h));
    undst = cv2.undistort(frame,mtx,dist, None, newcameramtx);
    x,y,w,h = roi;
    undst = undst[y:y+h, x:x+w];
    
    cv2.imshow("undistorted", undst);
    cv2.imshow("cap",frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

tot_error = 0;
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print ("total error: ", tot_error/len(objpoints))

cap.release()
cv2.destroyAllWindows()