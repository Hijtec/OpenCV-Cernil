'''
Created on 1. 11. 2017

@author: Hijtec
'''

import cv2
import numpy as np
import os

def transform(pos):
# This function is used to find the corners of the object and the dimensions of the object
    pts=[]
    n=len(pos)
    for i in range(n):
        pts.append(list(pos[i][0]))
       
    sums={}
    diffs={}
    tl=tr=bl=br=0
    for i in pts:
        x=i[0]
        y=i[1]
        sum=x+y
        diff=y-x
        sums[sum]=i
        diffs[diff]=i
    sums=sorted(sums.items())
    diffs=sorted(diffs.items())
    n=len(sums)
    rect=[sums[0][1],diffs[0][1],diffs[n-1][1],sums[n-1][1]]
    #      top-left   top-right   bottom-left   bottom-right
   
    h1=np.sqrt((rect[0][0]-rect[2][0])**2 + (rect[0][1]-rect[2][1])**2)     #height of left side
    h2=np.sqrt((rect[1][0]-rect[3][0])**2 + (rect[1][1]-rect[3][1])**2)     #height of right side
    h=max(h1,h2)
   
    w1=np.sqrt((rect[0][0]-rect[1][0])**2 + (rect[0][1]-rect[1][1])**2)     #width of upper side
    w2=np.sqrt((rect[2][0]-rect[3][0])**2 + (rect[2][1]-rect[3][1])**2)     #width of lower side
    w=max(w1,w2)
   
    return int(w),int(h),rect

cap = cv2.VideoCapture(0);
while True:
    ret, frame = cap.read();
 
    cv2.imshow('INPUT',frame)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray=cv2.GaussianBlur(gray,(11,11),0)
    edge=cv2.Canny(gray,100,200)
    _,contours,_=cv2.findContours(edge.copy(),1,1)
    cv2.drawContours(frame,contours,-1,[0,255,0],2)
    cv2.imshow('Contours',frame)
    n=len(contours)
    max_area=0
    pos=0
    a=0
    for i in contours:
        area=cv2.contourArea(i)
        if area>max_area:
            max_area=area
            pos=i
            a+=1
            peri=cv2.arcLength(pos,True)
            approx=cv2.approxPolyDP(pos,0.02*peri,True)
 
            size=frame.shape
            if a >= 5:
                w,h,arr=transform(approx)
 
                pts2=np.float32([[0,0],[w,0],[0,h],[w,h]])
                pts1=np.float32(arr)
                M=cv2.getPerspectiveTransform(pts1,pts2)
                dst=cv2.warpPerspective(frame,M,(w,h))
                image=cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
                #image=cv2.adaptiveThreshold(image,255,1,0,11,2)
                image = cv2.resize(image,(w,h),interpolation = cv2.INTER_AREA)
                cv2.imshow('OUTPUT',image)
                qcv2.waitKey(0)
            cv2.destroyAllWindows()
            
            
    processors = CPU_cascade.detectMultiScale(
            gray,               #ve kterém obrazu hledat
            minNeighbors=8,     #minimální počet blízkých pozitivních nálezů
            minSize=(70,70),    #minimální velikost nalezeného objektu
            maxSize=(80,80));   #maximální velikost nalezeného objektu
    for (cx,cy,cw,ch) in processors:
        cv2.rectangle(img, (cx,cy), (cx+cw, cy+ch), (0,0,255), 5);
        #vzkreslí obdelník na oblast detekovaného objektu červené barvy čarou tlustou 5px
        
