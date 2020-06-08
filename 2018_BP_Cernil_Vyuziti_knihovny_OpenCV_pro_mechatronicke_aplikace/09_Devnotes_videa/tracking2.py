'''
Created on 24. 10. 2017
Motion Tracking
@author: Martin Černil
'''
"""
Tento script pozoruje změny, následně tyto změny zaznamená a sleduje je pohyb-
ující se po obraze. Využito sledovacích algoritmů KCF a MIL.

Vytvořeno v rámci BP na fakultě FSI v Brně.
"""

"""
PSEUDOKÓD
1.Načti kameru + kalibrace
2.Snímek pozadí
3.Porovnej s aktuálním snímkem
4.Při první změně počkej daný čas, následně diferenci označ obdélníkem
5.Daný obdélník sleduj algorimty KCF/MIL
6.Uživatelský feedback
7.Zaznamenej dráhu obdélníku v čase
8.Sledování více objektů?
"""

import cv2,time,sys
import numpy as np

cap = cv2.VideoCapture(0);
timer = time.time(); #automaticky po nějakém čase bude generovat nové pozadí se kterým porovnává
x,y,w,h = 0,0,0,0; #pro čtverec obklopující objekt
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (640,480))
out2 = cv2.VideoWriter('output2.avi',fourcc, 30.0, (640,480))


while (True):
    ret, frame = cap.read();
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
    gray = cv2.GaussianBlur(gray, (21, 21), 0); # pro snížení noise
    
    try:
        firstFrame
    except(NameError):
        firstFrame = gray;     #vytvoření pozadí, které se nebude přepisovat při každém cyklu
        
    frameDelta = cv2.absdiff(firstFrame, gray); #diference mezi pozadím a aktuálním obrazem
    thresh = cv2.threshold(frameDelta, 50, 255, cv2.THRESH_BINARY)[1]; #přes threshold zvýrazním změny obrazu a převedu je na "data" pro findContours
    thresh = cv2.dilate(thresh, None, iterations=2);
    (__,cnts,___) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    
    for c in cnts:
        # pokud je contourArea moc malá, nevytvoří se obdélník
        if cv2.contourArea(c) < 2000:
            continue
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        
    if time.time()>timer+5:
        break   
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    cv2.imshow("delta", frameDelta);    
    cv2.imshow("threshold", thresh);
    cv2.imshow("tracking-prepare", frame);

cv2.destroyAllWindows    
tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
tracker_type = tracker_types[2]
    
if tracker_type == 'BOOSTING':
    tracker = cv2.TrackerBoosting_create()
if tracker_type == 'MIL':
    tracker = cv2.TrackerMIL_create()
if tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create()
if tracker_type == 'TLD':
    tracker = cv2.TrackerTLD_create()
if tracker_type == 'MEDIANFLOW':
    tracker = cv2.TrackerMedianFlow_create()
if tracker_type == 'GOTURN':
    tracker = cv2.TrackerGOTURN_create()
    
bbox = (x,y,w,h)
ret, frame = cap.read();
ok = tracker.init(frame, bbox)
mask = np.zeros(frame.shape, dtype = "uint8") #maska pro trajektorii

while True:
        ok, frame = cap.read()
        if not ok:
            break
         
        # Start timeru
        timer = cv2.getTickCount()
 
        # Update trackeru
        ok, bbox = tracker.update(frame)
 
        # Vypocet FPS (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
 
        # Vykreslení obklopujiciho obdelniku
        if ok:
            # Uspesny tracking
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking fail
            cv2.putText(frame, "Trackovani neuspesne", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
 
        # Zobrazení typu trackeru
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
     
        # Zobrazení FPS
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
 
        #Trajektorie
        xt = int(bbox[0]+bbox[2]/2)
        yt = int(bbox[1]+bbox[3]/2)
        cv2.circle(mask, (xt,yt), 8, (255,255,255), -1)
        maskedImg = cv2.bitwise_and(frame,mask)
        maskedImg = cv2.dilate(maskedImg, None, iterations=2);
        
        # Výsledek
        cv2.imshow("Tracking", frame)
        cv2.imshow("Trajectory", maskedImg)
        out.write(frame)        #vystup trackingu do avi
        out2.write(maskedImg)   #vystup trajektorie do avi
        # Ukončení
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
    
cap.release();
cv2.destroyAllWindows();