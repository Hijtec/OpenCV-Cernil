'''
Created on 25. 10. 2017
funkce pro zjednodušení/zpřehlednění skriptů


@author: Hijtec
'''
import numpy as np
import cv2, time


    
def Release(): #odpojí kameru a zavře všechny okna
    try:
        cap.release();
    except:
        pass
    try:
        cap2.release();
    except:
        pass
    cv2.destroyAllWindows()


def CalibrateCamera(camera): #vrátí feed, matici, přetvoření a další věci, celkem 7
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    objpoints = []
    imgpoints = []

    cap = cv2.VideoCapture(camera);
    i = 0
    
    while i<=20:
        __, frame = cap.read();
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);
        ret, corners = cv2.findChessboardCorners(gray, (9,6),None);
    
        if ret == True:
            objpoints.append(objp);
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria);
            imgpoints.append(corners2);
            frame = cv2.drawChessboardCorners(frame, (9,6), corners2,ret);
            i +=1;
            time.sleep(1);
        
    
        cv2.imshow("cap",frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None);
    
    return cap, mtx, dist, rvecs, tvecs, objpoints, imgpoints
    cv2.waitKey(1);
    cv2.destroyWindow('cap'); #tohle se nechce zavřít :D

def Calibrate2Cameras(camera1,camera2): #vrátí feed, matici, přetvoření a další věci, celkem 7
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    objp2 = np.zeros((6*9,3), np.float32)
    objp2[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    objpoints = []
    imgpoints = []
    objpoints2 = []
    imgpoints2 = []
    
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

    cap = cv2.VideoCapture(camera1);
    cap2 = cv2.VideoCapture(camera2);
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    i = 0
    
    while i<=50:
        __, frame = cap.read();
        __, frame2 = cap2.read();
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);
        gray2 = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY);
        
        ret, corners = cv2.findChessboardCorners(gray, (9,6),None);
        ret2, corners2 = cv2.findChessboardCorners(gray2, (9,6),None);
        
        both = ret + ret2;
    
        if both == 2:
            objpoints.append(objp);
            objpoints2.append(objp2);
            
            corners_draw = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria);
            corners2_draw2 = cv2.cornerSubPix(gray2,corners2,(11,11),(-1,-1),criteria);
            
            imgpoints.append(corners_draw);
            imgpoints2.append(corners2_draw2);
            
            frame = cv2.drawChessboardCorners(frame, (9,6), corners_draw,ret);
            frame2 = cv2.drawChessboardCorners(frame2, (9,6), corners2_draw2,ret2);
            
            i +=1;
            time.sleep(0.5);
        
        
        cv2.imshow("cap",frame)
        cv2.imshow("cap2",frame2)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None);
    ret2,mtx2,dist2,rvecs2,tvecs2 = cv2.calibrateCamera(objpoints2,imgpoints2,gray2.shape[::-1],None,None);
    imgsize=gray.shape[::-1]
    print(imgsize)
    
    
    (_,_,_,_,_,rotationMatrix, translationVector,_,_) = cv2.stereoCalibrate(objpoints,imgpoints,imgpoints2,mtx,dist,mtx2,dist2,gray.shape[::-1],None,None,cv2.CALIB_FIX_INTRINSIC, criteria)
    (leftRectification, rightRectification, leftProjection, rightProjection, dispartityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(mtx, dist,mtx2, dist2,gray.shape[::-1], rotationMatrix, translationVector, cv2.CALIB_ZERO_DISPARITY, 0,(0,0))
    
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(mtx, dist, leftRectification, leftProjection, imgsize, cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(mtx2, dist2, rightRectification, rightProjection, imgsize, cv2.CV_32FC1)
    
    
    
    cv2.destroyWindow('cap'); #tohle se nechce zavřít :D
    cv2.destroyWindow('cap2'); #tohle se nechce zavřít :D
    
    return imgsize, leftMapX, leftMapY, leftROI, rightMapX, rightMapY, rightROI, frame, frame2

def Undistort(cap, mtx, dist): #vrátí zkalibrovaný feed kamery
    while True:
        if cap == 0:
            cap = cv2.VideoCapture(0)
        else:
            cap = cv2.VideoCapture(1)
        
        ___,frame = cap.read()
        h, w = frame.shape[:2];
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h));
        undst = cv2.undistort(frame,mtx,dist, None, newcameramtx);
        x,y,w,h = roi;
        undst = undst[y:y+h, x:x+w];
        
        return undst