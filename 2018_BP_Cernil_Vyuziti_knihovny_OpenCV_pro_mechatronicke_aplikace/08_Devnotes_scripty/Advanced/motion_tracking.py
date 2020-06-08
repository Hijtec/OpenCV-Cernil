'''
Created on 24. 10. 2017
Motion Tracking
@author: Hijtec
'''
import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0);
oldtime = float(0); #automaticky po nějakém čase bude generovat nové pozadí se kterým porovnává
xmin,ymin,xmax,ymax = 0,0,0,0

while (True):
    ret, frame = cap.read();
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    gray = cv2.GaussianBlur(gray, (21, 21), 0); # pro snížení noise
    
    try:
        firstFrame
    except(NameError):
        firstFrame = gray;     #vytvoření pozadí, které se nebude přepisovat při každém cyklu
        
    frameDelta = cv2.absdiff(firstFrame, gray); #diference mezi pozadím a aktuálním obrazem
    thresh = cv2.threshold(frameDelta, 15, 255, cv2.THRESH_BINARY)[1]; #přes threshold zvýrazním změny obrazu a převedu je na "data" pro findContours
    thresh = cv2.dilate(thresh, None, iterations=2);
    (__,cnts,___) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    for c in cnts:
        # pokud je contourArea moc malá, nevytvoří se obdélník
        if cv2.contourArea(c) < 5000:
            continue
        
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,(0,0,255),2)

        xmin = int(1000)
        ymin = int(1000)
        xmax = int(0)
        ymax = int(0)
        for i in box:
            if xmin > i[0]:
                xmin = i[0]
            if ymin > i[1]:
                ymin = i[1]
            if xmax < i[0]:
                xmax = i[0]
            if ymax < i[1]:
                ymax = i[1]
        if xmin < 0:
            xmin = 0
        if ymin < 0:
            ymin = 0
        
        cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (255,0,0), 5);    
    
    cv2.imshow("delta", frameDelta);    
    cv2.imshow("threshold", thresh);   
    cv2.imshow("tracking", frame);
    print(xmin,xmax,ymin,ymax)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();