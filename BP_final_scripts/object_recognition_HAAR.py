"""
Tento skript rozpoznává objekty, jež byly natrénovany předem na pozitivních
a negativních snímcích. Skript využívá inventářní systém, detekuje který objekt
chybí a na tuto skutečnost uživatele upozorňuje.

PSEUDOKÓD
0.HAAR trénink detekovaných objektů
1.Načti kameru
2.Rozpoznání objektů
3.Označení objektů na obraze
4.Učení polohy
5.Uživatelský feedback (chybí/nemůže najít, špatná detekce atp.)
"""

import cv2, time
import numpy as np

""""Vstupní parametry + načtení kaskád"""
klice = cv2.CascadeClassifier("haarcascades/klice_17.xml"); # objekt 1
nuzky = cv2.CascadeClassifier("haarcascades/nuzky.xml"); # objekt 2
cap = cv2.VideoCapture(0);
truth_1= 1; truth_2= 1; #
timer1 = time.time()+100000; timer2 = time.time()+100000 # časovače
out = cv2.VideoWriter('output_detection.avi',cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640,480))
        # nahrávání skriptu

"""Detekce objektu"""
def detect(frame, minSize, minNeighbors, groupRectangleMin, esq, cascade, truth, timer):
    #minSize = nejmenší možný obdélník
    #minNeighbors = nejmenší možný počet sousedících obdélníků
    #groupRectangleMin = nejmenší možný počet sjednocených obdélníků
    #esq = 1-esq procentuální překrytí sousedících obdélníků pro jejich sjednocení
    #cascade = vybraná haarova kaskáda pro detekci
    item = cascade.detectMultiScale(frame, minSize=minSize, minNeighbors=minNeighbors);
    item, weights_item = cv2.groupRectangles(np.array(item).tolist(), groupRectangleMin, esq)   
    """Spojení nejvíce možných obdélníků, potlačení ostatních"""
    try:
        weights_item = weights_item[0];
        if weights_item[0]>=truth:
            ok = 1;
            truth = weights_item[0];
            marker = item[0];
            timer = time.time()
            if time.time()>timer+3:
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
    tick = cv2.getTickCount(); # funkce pro výpočet FPS
    cv2.line(frame, (int(np.shape(frame)[1]*0.33),0), (int(np.shape(frame)[1]*0.33),int(np.shape(frame)[0])), (255,255,0), 10);  
    cv2.line(frame, (int(np.shape(frame)[1]*0.66),0), (int(np.shape(frame)[1]*0.66),int(np.shape(frame)[0])), (255,255,0), 10);
    """Detekce objektu 1"""
    marker_1, truth_1, ok_1, timer1 = detect(frame, (30,30), 1, 1, 0.5, klice, truth_1, timer1);
    if ok_1 == 1:
        kx1,ky1,kw1,kh1 = marker_1;
        cv2.rectangle(frame, (kx1,ky1), (kx1+kw1,ky1+kh1), (0,0,0), 8); #vykreslení obdélníku
        xt1=int(kx1+kw1/2) #x-ová souřadnice těžiště obrazce
        yt1=int(ky1+kh1/2) #y-ová souřadnice těžiště obrazce
        cv2.circle(frame, (xt1,yt1), 10, (0,0,255), -1); #vykreslení středu
        cv2.putText(frame, "KLICE", (kx1,ky1), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,250,250),2);
    elif time.time()>timer1+2:
        cv2.putText(frame, "Detection 1 failed ", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,50,250),2); #selhání detekce
    """Inventář objektu 1"""

    try:
        if xt1 < (np.shape(frame)[1]*0.33): #specifikace místa těžiště
            cv2.putText(frame, "Objekt 1 poloha OK", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        else:
            cv2.putText(frame, "Objekt 1 poloha notOK", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    except:
        pass
    """Detekce objektu 2"""
    marker_2, truth_2, ok_2, timer2 = detect(frame, (50,50), 2, 1, 0.99, nuzky, truth_2, timer2);
    if ok_2 == 1:
        kx2,ky2,kw2,kh2 = marker_2;
        cv2.rectangle(frame, (kx2,ky2), (kx2+kw2,ky2+kh2), (255,0,0), 8); #vykreslení obdélníku
        xt2=int(kx2+kw2/2) #x-ová souřadnice těžiště obrazce
        yt2=int(ky2+kh2/2) #y-ová souřadnice těžiště obrazce
        cv2.circle(frame, (xt2,yt2), 10, (0,0,255), -1); #vykreslení středu
        cv2.putText(frame, "NUZKY", (kx2,ky2), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,250,250),2);
    elif time.time()>timer2+0.5:
        cv2.putText(frame, "Detection 2 failed", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,50,250),2); #selhání detekce
    """Inventář objektu 2"""
    
    try:
        if xt2 > (np.shape(frame)[1]*0.66): #specifikace místa těžiště
            cv2.putText(frame, "Objekt 2 poloha OK", (100,140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        else:
            cv2.putText(frame, "Objekt 2 poloha notOK", (100,140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    except:
        pass
    
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - tick); #výpočet FPS
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    # Zobrazení FPS
    cv2.imshow("img", frame)
    out.write(frame)            # výstup trackingu do .avi
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release();
cv2.destroyAllWindows();