"""
Tento skript po zkalibrování páru kamer nabídne k nahlédnutí disparity mapu,
pomocí které se za pomocí kalibračního obrazce nakonfiguruje nebezpečná a 
kritická vzdálenost. Tyto stavy jsou následně zobrazeny pro uživatelský feedback.

PSEUDOKÓD
1.Načti 2 kamery + stereokalibrace
2.Výpočet a kalibrace disparity mapy
3.Kalibrace bezpečné vzdálenosti pomocí specifického objektu
4.Uživatelský feedback (stav vzdálenosti)
5.Output (Objekt v bezpečné/nebezpečné oblasti, FPS atd.)
5.Zaznamenání času prostoje
"""

import numpy as np
import cv2, time

"""Kalibrace Kamery"""
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
    
    #Nastavení kamery
    CAMERA_WIDTH = 720
    CAMERA_HEIGHT = 640
    CAMERA_BRIGHTNESS = 5

    cap = cv2.VideoCapture(camera1);
    cap2 = cv2.VideoCapture(camera2);
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, CAMERA_BRIGHTNESS)
    cap2.set(cv2.CAP_PROP_BRIGHTNESS, CAMERA_BRIGHTNESS)
    i = 0
    
    while i<=30:
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
            
            i += 1;
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
def new(imgL,imgR):
    #výpočet depthmap
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0;
    cv2.imshow('left', imgL);
    print ('computing disparity...');
    
    disparity = ((disp-min_disp)/num_disp);
    erosion = cv2.erode(disparity,kernel_erode,iterations = 1);
    dilatation = cv2.dilate(erosion,kernel_dilate,iterations = 1);
    blur = cv2.GaussianBlur(dilatation,(3,3),0);
    cv2.imshow("disparity",disparity);
    cv2.imshow('tweaked', blur);
        
    return dilatation
    time.sleep(0.05)
def feed(cap1,cap2):
    ret1, frame1 = cap1.read();
    ret2, frame2 = cap2.read();
    #Transformace kamer skrz 3D kalibraci
    frame1 = cv2.remap(frame1, leftMapX, leftMapY, cv2.INTER_LINEAR); # undistorts the image for further work with it
    frame2 = cv2.remap(frame2, rightMapX, rightMapY, cv2.INTER_LINEAR ); # undistorts the image for further work with it
    return frame1,frame2
def update(val = 0):
    stereo.setBlockSize(cv2.getTrackbarPos('window_size', 'disparity'))
    stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio', 'disparity'))
    stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize', 'disparity'))
    stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange', 'disparity'))
    stereo.setDisp12MaxDiff(cv2.getTrackbarPos('disp12MaxDiff', 'disparity'))
    stereo.setP1(cv2.getTrackbarPos('P1', 'disparity'))
    stereo.setP2(cv2.getTrackbarPos('P2', 'disparity'))
def diff(firstFrame,dispmap,roi):
    
    frameDelta = cv2.absdiff(firstFrame, dispmap);
    frameDelta = np.uint8(frameDelta*255)
    thresh = cv2.threshold(frameDelta, 50, 245, cv2.THRESH_BINARY)[1];
    frameDeltaROI = frameDelta[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
    dispmapROI = np.uint8(dispmap*255)
    dispmapROI = dispmapROI[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
    
    return frameDeltaROI, dispmapROI
def calibrate2safedistance(frameDeltaROI,dispmapROI):
    print(cv2.mean(frameDeltaROI))
    print(cv2.mean(dispmapROI))
    if cv2.waitKey(200) & 0xFF == ord("s"):
        savedMeanDelta = cv2.mean(frameDeltaROI)
        savedMeanDispmap = cv2.mean(dispmapROI)
        return savedMeanDelta, savedMeanDispmap
    else:
        return 0,0

if __name__ == "__main__":
    #Nastavení depthmap
    window_size = 5
    min_disp = -64
    num_disp = 192
    uniquenessRatio = 1
    speckleRange = 2
    speckleWindowSize = 75
    disp12MaxDiff = 50
    P1 = 600
    P2 = 2400
    
    cv2.namedWindow('disparity')
    cv2.createTrackbar('speckleRange', 'disparity', speckleRange, 50, update)    
    cv2.createTrackbar('window_size', 'disparity', window_size, 21, update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', speckleWindowSize, 200, update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', uniquenessRatio, 50, update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', disp12MaxDiff, 250, update)
    cv2.createTrackbar('P1', 'disparity', P1, 600, update)
    cv2.createTrackbar('P2', 'disparity', P2, 2400, update)
    
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp,        #minimální možná disparita(nesourodost)
        numDisparities = num_disp,      #maximální možná disparita(nesourodost)
        blockSize = window_size,        #velikost oblasti ke spárování
        uniquenessRatio = uniquenessRatio, #procentuální rozpětí pro potvrzení správnosti párování
        speckleRange = speckleRange,    #maximální variace disparity v každém spojitém prvku
        speckleWindowSize = speckleWindowSize, #maximální velikost spojité oblasti, kdy ještě šum v této oblasti bude anulován
        disp12MaxDiff = disp12MaxDiff,  #maximální diference mezi pixely obou snímků
        P1 = P1,                        #trest změny sousedících pixelů o 1 hodnotu
        P2 = P2                         #trest změny sousedících pixelů o více než 1 hodnotu
        )

(imgsize, leftMapX, leftMapY, leftROI, rightMapX, 
rightMapY, rightROI, frame, frame2 )= Calibrate2Cameras(1,2);
cap1 = cv2.VideoCapture(1);
cap2 = cv2.VideoCapture(2);
kernel_erode = np.ones((7,7),np.uint8)
kernel_dilate = np.ones((5,5),np.uint8)
savedMeanDelta_danger = 0
savedMeanDelta_stop = 0


while True:
    timer = cv2.getTickCount();
    img = np.zeros((512,512,3), np.uint8)
    
    imgL,imgR = feed(cap1,cap2);
    dispmap = new(imgL, imgR);
    
    try:
        firstFrame
    except(NameError):
        firstFrame = dispmap;
    try:
        roi
    except(NameError):
        roi = cv2.selectROI(dispmap)
    
    frameDeltaROI, dispmapROI = diff(firstFrame, dispmap,roi);
    
    while savedMeanDelta_danger == 0:
        imgL,imgR = feed(cap1,cap2);
        dispmap = new(imgL, imgR);
        frameDeltaROI, dispmapROI = diff(firstFrame, dispmap,roi);
        cv2.imshow("dispmap-calibrate", dispmapROI)
        cv2.putText(dispmapROI, "Kalibrace DANGER", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        savedMeanDelta_danger, savedMeanDispmap_danger = calibrate2safedistance(frameDeltaROI, dispmapROI);
        savedMeanDelta_danger = savedMeanDelta_danger;
            
    while savedMeanDelta_stop == 0:
        imgL,imgR = feed(cap1,cap2);
        dispmap = new(imgL, imgR);
        frameDeltaROI, dispmapROI = diff(firstFrame, dispmap,roi);
        cv2.imshow("dispmap-calibrate", dispmapROI)
        cv2.putText(dispmapROI, "Kalibrace STOP", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        savedMeanDelta_stop, savedMeanDispmap_danger = calibrate2safedistance(frameDeltaROI, dispmapROI);
        savedMeanDelta_stop = savedMeanDelta_stop;
            
    if savedMeanDelta_danger <  cv2.mean(frameDeltaROI):
        cv2.putText(img, "DANGER oblast", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),2);
        if savedMeanDelta_stop < cv2.mean(frameDeltaROI):
            cv2.putText(img, "STOP oblast", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255),2);
            
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    cv2.putText(dispmapROI, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    
    #Výsledek
    cv2.imshow("delta", frameDeltaROI)
    cv2.imshow("dispmap-small", dispmapROI);
    cv2.imshow("info", img)
    
    if cv2.waitKey(2) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
cap1.release()
cap2.release()