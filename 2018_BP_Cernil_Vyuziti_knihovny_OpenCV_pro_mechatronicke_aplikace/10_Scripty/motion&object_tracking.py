"""
Tento script pozoruje změny, následně tyto změny zaznamená a sleduje je pohyb-
ující se po obraze. Využito sledovacích algoritmů KCF a MIL (dostupných více).

Vytvořeno v rámci BP na fakultě FSI v Brně. 20.4.2018
"""

"""
PSEUDOKÓD
1.Načti kameru
2.Snímek pozadí
3.Porovnej s aktuálním snímkem
4.Při první změně počkej daný čas, následně diferenci označ obdélníkem
5.Daný obdélník sleduj algoritmy KCF/MIL
6.Uživatelský feedback
7.Zaznamenej dráhu obdélníku v čase
8.Pracuj s touto hodnotou pro počítání objektů (počet, pozice,...)
"""

import cv2,time
import numpy as np

def target(cap, out2):
    while (True):
        ____, frame = cap.read();
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
        gray = cv2.GaussianBlur(gray, (21, 21), 0); # pro snížení noise
    
        """Vytvoření snímku pro porovnání s aktuálním feedem"""
        try:
            firstFrame
        except(NameError):
            firstFrame = gray;  #vytvoření pozadí, které se nebude přepisovat při každém cyklu
    
        """Porovnání s prvním snímkem + uprava"""    
        frameDelta = cv2.absdiff(firstFrame, gray); #diference mezi pozadím a aktuálním obrazem
        #přes threshold zvýrazním změny obrazu a převedu je na "data" pro findContours
        thresh = cv2.threshold(frameDelta, 40, 255, cv2.THRESH_BINARY)[1]; 
        thresh = cv2.dilate(thresh, None, iterations=1);
        
        """Hledání souvislých změn pro označení"""
        (__,cnts,___) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
        for c in cnts:
        # pokud je contourArea moc malá nebo velká, nevytvoří se obdélník
            if cv2.contourArea(c) < 10000:
                continue
            if cv2.contourArea(c) > 50000:
                continue
            x,y,w,h = cv2.boundingRect(c) # kolem kontury se vytvoří nejmenší čtverec
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        
        cv2.imshow("delta", frameDelta);    
        cv2.imshow("threshold", thresh);
        cv2.imshow("tracking-prepare", frame);
        out2.write(frame)
        if cv2.waitKey(2) & 0xFF == ord("q"): 
            # stisknutím klávesy "q" se hledání změny zastaví
            break
        
        """Detekce změny + spuštění časovače"""
        try:
            timerDiff
        except(NameError):
            timer = time.time()+10000;
            try:
                if x!=0:
                    timer = time.time();
                    timerDiff=timer
                    
            except:
                continue
    
        """Přerušení po zadaném čase"""
        if time.time()>timer+0.5:
            break       
    return x,y,w,h, frame

"""Výběr a inicializace trackeru"""
def createTracker(trackerType):
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[trackerType]
    
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
    return tracker, tracker_type

"""Sledování objektu"""
def Track(tracker,frame):
    ok, bbox = tracker.update(frame)
    p1, p2 = False, False
    if ok:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else:
        cv2.putText(frame, "Trackovani neuspesne", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2);
    return frame, ok, p1, p2

"""Vstupní parametry"""
pocet = 0;
chyba = 0;
cap = cv2.VideoCapture(1); # video vstup
____, frame = cap.read();
out = cv2.VideoWriter('output_tracking.avi',cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640,480))
        # nahrávání skriptu
out2 = cv2.VideoWriter('output_tracking2.avi',cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640,480))
        # nahrávání skriptu
"""Volání funkcí"""
x,y,w,h, frame_init = target(cap, out2);
bbox = (x,y,w,h);
tracker, tracker_type = createTracker(2);
tracker.init(frame, bbox);

"""Kontinuální sledování"""
while (True):
    ____, frame = cap.read();
    timer = cv2.getTickCount(); # funkce pro výpočet FPS
    frame, ok, p1, p2 = Track(tracker, frame);
    
    if ok == False:
        cv2.putText(frame, "Trackovani neuspesne", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2);
        cv2.imshow("output", frame);
        
        """Detekce chyby ve střední části obrazu"""
        try:
            if xt > (np.shape(frame)[1]*0.25):
                if xt < (np.shape(frame)[1]*0.75):
                    chyba += 1
        except:
            pass
        
        first = time.time();
        cap.release();
        time.sleep(1); # pozastavení skriptu
        cap = cv2.VideoCapture(1);
        
        """Reinicializace sledování"""
        x,y,w,h, frame_init = target(cap, out2);
        bbox = (x,y,w,h);
        tracker, tracker_type = createTracker(2);
        ____, frame = cap.read();
        tracker.init(frame, bbox);
        continue
    

    xt = int((p1[0]+p2[0])/2) # x-ová souřadnice těžiště sledovaného objektu
    yt = int((p1[1]+p2[1])/2) # y-ová souřadnice těžiště sledovaného objektu
    
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer); #výpočet FPS
    
    """Zapamatování si počáteční pozice objektu"""
    try:
        pozice1
    except(NameError):
        pozice1 = xt
    
    """Rozhodnutí mezi pohyb z jedné či druhé strany"""
    if pozice1 < (np.shape(frame)[1]/2):
        cv2.putText(frame, "Prichod", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        if xt > (np.shape(frame)[1]*0.75):
            pocet += 1;
            del pozice1
        elif xt < (np.shape(frame)[1]*0.25):
            del pozice1
    else:
        cv2.putText(frame, "Odchod", (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
        if xt < (np.shape(frame)[1]*0.25):
            pocet -= 1;
            del pozice1
        elif xt > (np.shape(frame)[1]*0.75):
            del pozice1
    
    
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    # Zobrazení typu trackeru
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    # Zobrazení FPS
    cv2.putText(frame, "Chyby :" + str(chyba), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2);
    # Zobrazení neúspěšnosti
    cv2.putText(frame, str(pocet), (int((np.shape(frame)[1]/2)),50), cv2.FONT_HERSHEY_SIMPLEX, 2, (50,170,50),2);
    # Zobrazení poctu objektů
    
    out.write(frame)            # výstup trackingu do .avi
    cv2.imshow("output", frame) # výstupní obraz
    
    if cv2.waitKey(2) & 0xFF == ord("q"):
        # konec programu po stisknutí klávesy "q"
        cap.release()
        break
    
