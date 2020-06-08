'''
Created on 9. 10. 2017
Using Haar Cascades to detect various objects
@author: Hijtec
'''
import cv2, time
import numpy as np


klice = cv2.CascadeClassifier("haarcascades/klice_nove.xml");
penezenka = cv2.CascadeClassifier("haarcascades/kondenzator.xml");
napajeni_cascade = cv2.CascadeClassifier("haarcascades/napajeni_velke.xml");

cap = cv2.VideoCapture(0);
truth_1= 0;
truth_2= 0;
truth_3= 0;
timer1 = time.time()+100000
timer2 = time.time()+100000
timer3 = time.time()+100000

def detect(frame, minSize, minNeighbors, groupRectangleMin, esq, cascade, truth, timer):
    
    item = cascade.detectMultiScale(frame, minSize=minSize, minNeighbors=minNeighbors);
    item, weights_item = cv2.groupRectangles(np.array(item).tolist(), groupRectangleMin, esq)   
    try:
        weights_item = weights_item[0];
        if weights_item[0]>truth:
            ok = 1;
            truth = weights_item[0];
            marker = item[0];
            timer = time.time()
            if time.time()>timer+1:
                truth = 0;
            return marker, truth, ok, timer
        else:
            return 0,0,0,timer
    except:
        ok = 0;
        marker = 0;
        truth = 0;
        return marker, truth, ok, timer

while (True):
    ret, frame = cap.read();
    
    """detekce objektu 1"""
    marker_1, truth_1, ok_1, timer1 = detect(frame, (80,80), 2, 2, 0.99, klice, truth_1, timer1);
    if ok_1 == 1:
        kx1,ky1,kw1,kh1 = marker_1;
        cv2.rectangle(frame, (kx1,ky1), (kx1+kw1,ky1+kh1), (0,0,0), 8);
        x1=int(kx1+kw1/2)
        y1=int(ky1+kh1/2)
        cv2.circle(frame, (x1,y1), 10, (0,0,255), -1); #where, middle, radius, color, width (-1 equals to filled circle)
    elif time.time()>timer1+0.5:
        cv2.putText(frame, "Detection 1 failed ", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,50,250),2);
    
    try:
        if x1 < (np.shape(frame)[1]*0.33):
            cv2.putText(frame, "Objekt 1 OK", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        else:
            cv2.putText(frame, "Objekt 1 not OK", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    except:
        pass
    
    """detekce objektu 2"""
    marker_2, truth_2, ok_2, timer2 = detect(frame, (80,80), 8, 2, 0.99, penezenka, truth_2, timer2);
    if ok_2 == 1:
        kx2,ky2,kw2,kh2 = marker_2;
        cv2.rectangle(frame, (kx2,ky2), (kx2+kw2,ky2+kh2), (255,0,0), 8);
        x2=int(kx2+kw2/2)
        y2=int(ky2+kh2/2)
        cv2.circle(frame, (x2,y2), 10, (0,0,255), -1); #where, middle, radius, color, width (-1 equals to filled circle)
    elif time.time()>timer2+0.5:
        cv2.putText(frame, "Detection 2 failed", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,50,250),2);
    
    try:
        if x2 < (np.shape(frame)[1]*0.66):
            if x2 > (np.shape(frame)[1]*0.33):
                cv2.putText(frame, "Objekt 2 OK", (100,140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        else:
            cv2.putText(frame, "Objekt 2 not OK", (100,140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    except:
        pass
             
    """detekce objektu 3"""
    marker_3, truth_3, ok_3, timer3 = detect(frame, (80,80), 8, 2, 0.99, napajeni_cascade, truth_3, timer3);
    if ok_3 == 1:
        kx3,ky3,kw3,kh3 = marker_3;
        cv2.rectangle(frame, (kx3,ky3), (kx3+kw3,ky3+kh3), (0,255,0), 8);
        x3=int(kx3+kw3/2)
        y3=int(ky3+kh3/2)
        cv2.circle(frame, (x3,y3), 10, (0,0,255), -1); #where, middle, radius, color, width (-1 equals to filled circle)
    elif time.time()>timer3+0.5:
        cv2.putText(frame, "Detection 3 failed", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,50,250),2);
    
    try:
        if x3 > (np.shape(frame)[1]*0.66):
            cv2.putText(frame, "Objekt 3 OK", (100,170), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        else:
            cv2.putText(frame, "Objekt 3 not OK", (100,170), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    except:
        pass
    
    cv2.imshow("img", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();
"""
while(True):
    ret, img = cap.read();
   
    klic = klice.detectMultiScale(img,minSize=(80,80),minNeighbors=2);
    klic, weights_klic = cv2.groupRectangles(np.array(kond).tolist(), 2, 0.99)
    #timer
    try:
        weights_klic[0]
    except:
        cv2.imshow("img", img)
        continue
    
    if weights[0]>truth:
        timer = time.time()
        truth = weights[0];
        kond = kond[0];
        print(truth)
        print(kond)
        kx,ky,kw,kh = kond;
    
    cv2.rectangle(img, (kx,ky), (kx+kw,ky+kh), (0,0,0), 8);
    
    if time.time()>timer+3:
        truth = 0;
    
    cv2.imshow("img", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();"""