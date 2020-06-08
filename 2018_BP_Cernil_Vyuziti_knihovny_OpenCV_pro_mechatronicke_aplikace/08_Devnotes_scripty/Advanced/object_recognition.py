'''
Created on 1. 11. 2017

@author: Hijtec
'''
import cv2
import numpy as np


"""TRAINING"""
clock = cv2.CascadeClassifier("haarcascades/clock.xml");
"""ImageExtraction"""
cap = cv2.VideoCapture(1);
storage = 100;
i = 1;

def auto_canny(image, sigma=0.5):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
 
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged

while (True):
    ret, frame = cap.read();
    blur = cv2.GaussianBlur(frame, (3, 3), 0); # pro snížení noise
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY);
    gray = cv2.dilate(gray, None, iterations=2);
    
    mask = np.zeros(frame.shape[:2], np.uint8);
    bgModel = np.zeros((1,65), np.float64);
    fgModel = np.zeros((1,65), np.float64);
    
    ooi = auto_canny(gray)
    (__,cnts,___) = cv2.findContours(ooi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE);
    for c in cnts:
        # pokud je contourArea moc malá, nevytvoří se obdélník
        if cv2.contourArea(c) < 1000:
            continue
        if cv2.contourArea(c) > 20000:
            continue
        
        # výpočet obdélníku pro pohyb, vykreslení do obrazu
        if cv2.contourArea(c) >= storage - 20:
            storage = cv2.contourArea(c)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(blur,(x,y),(x+w,y+h),(0,0,0),2)
            rect = (x,y,w,h);
            cv2.grabCut(frame, mask, rect, bgModel, fgModel, 3, cv2.GC_INIT_WITH_RECT); #not realtime, computationaly difficult, nutné nakalibrovat pro dané prostředí, integer upřest funkce zvyšuje přesnost ale také výpočetní zátěž
            mask2 = np.where((mask==2) | (mask==0), 0, 1).astype("uint8");
            cutted = frame*mask2[:,:,np.newaxis];
            cv2.imshow("grabcut",cutted);
            
            
            boundrect = cv2.minAreaRect(c)
            xy,wh,alfa = cv2.minAreaRect(c)
            box = cv2.boxPoints(boundrect)
            box = np.int0(box)
            mask2 = np.zeros((frame.shape[0], frame.shape[1]))
            
            cv2.fillConvexPoly(mask2, box, 1);
            mask2 = mask2.astype(np.bool);     
            out = np.zeros_like(frame);
            out[mask2] = frame[mask2] ;
            
            """nutné otočit výstup"""
            rows,cols = gray.shape
            rmatrix = cv2.getRotationMatrix2D(xy,alfa,1)
            rotated = cv2.warpAffine(out,rmatrix,(cols,rows))
            
            """a teď třeba extrahovat obrázek"""
            gray_rotated = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
            roi = auto_canny(gray_rotated)
            (__,cnts,___) = cv2.findContours(roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE);
            for c in cnts:
                x,y,w,h = cv2.boundingRect(c)
                roi = rotated[y:y+h,x:x+w];
                cv2.imshow("output", roi)
                continue
            
            if cv2.waitKey(1) & 0xFF == ord("f"):
                
                file = "test/p{a}.png".format(a=i);
                i+=1;
                cv2.imwrite(file, roi)
            
            cv2.imshow("cropped", out);
            cv2.imshow("rotate", rotated);
    
    cv2.imshow("frame", frame);
    cv2.imshow("ooi", ooi);
    cv2.imshow("orig", blur);
    
    
   
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break