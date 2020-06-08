'''
Created on 20. 4. 2018

@author: Hijtec
'''
import cv2,time
import numpy as np

def target(cap,lower,upper, out2):
    while (True):
        ____, frame = cap.read();
        """Vytvoření barevné masky"""
        mask = cv2.inRange(frame, lower, upper);
        mask = cv2.erode(mask,None, iterations = 3);
        mask = cv2.dilate(mask, None, iterations=5);
        try:
            firstMask
            mask = cv2.bitwise_and(mask,firstMask)
            cv2.imshow("maska", mask)
        except(NameError):
            firstMask = cv2.bitwise_not(mask)
        """Převod do černobíla"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
        gray = cv2.GaussianBlur(gray, (21, 21), 0); # pro snížení noise
        """Vytvoření snímku pro porovnání s aktuálním feedem"""
        try:
            firstFrame
        except(NameError):
            firstFrame = gray;  
            #vytvoření pozadí, které se nebude přepisovat při každém cyklu
        """Porovnání s prvním snímkem + uprava"""  
        frameDelta = cv2.absdiff(firstFrame, gray); 
        #diference mezi pozadím a aktuálním obrazem
        thresh = cv2.threshold(frameDelta, 40, 255, cv2.THRESH_BINARY)[1]; 
        #přes threshold zvýrazním změny obrazu a převedu je na "data" pro findContours
        thresh = cv2.dilate(thresh, None, iterations=1);
        cv2.imshow("change", thresh)
        """Maska + pohyb"""
        search = cv2.bitwise_and(mask,firstFrame) # binární sjednocení 
        cv2.imshow("lol", search)
        """Hledání souvislých změn pro označení"""
        (__,cnts,___) = cv2.findContours(search.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
        for c in cnts:
        # pokud je contourArea moc malá, nevytvoří se obdélník
            if cv2.contourArea(c) < 2000:
                continue
            if cv2.contourArea(c) > 10000:
                continue
            x,y,w,h = cv2.boundingRect(c) # kolem kontury se vytvoří nejmenší čtverec
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow("tracking-prepare", frame);
        out2.write(frame)
        if cv2.waitKey(2) & 0xFF == ord("q"):
            break
        """Detekce změny + spuštění časovače"""
        try:
            timerDiff
        except(NameError):
            timer = time.time()+10000;
            try:
                if x!=0:
                    timerDiff = time.time();
                    timer = timerDiff;
            except:
                continue
        """Přerušení po zadaném čase"""
        if time.time()>timer+0.5:
            # stisknutím klávesy "q" se hledání změny zastaví
            break       
    return x,y,w,h
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
"""Vstupní parametry a proměnné"""
lower = np.array((0,0,0), dtype="uint8"); upper = np.array((60,60,60), dtype="uint8")
counter_left = 0;counter_right = 0;
chyba = 0; state = 0; cap = cv2.VideoCapture(0);
out = cv2.VideoWriter('output_PONG.avi',cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640,480))
out2 = cv2.VideoWriter('output_PONG2.avi',cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640,480))
"""Volání funkcí"""
x,y,w,h = target(cap,lower,upper, out2);
bbox = (x,y,w,h);
tracker, tracker_type = createTracker(2);
____, frame = cap.read();
tracker.init(frame, bbox);

while (True):
    ____, frame = cap.read();
    timer = cv2.getTickCount();
    frame, ok, p1, p2 = Track(tracker, frame);
    if ok == False:
        cv2.putText(frame, "Trackovani neuspesne", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2);
        cv2.imshow("output", frame);
        try:
            if xt > (np.shape(frame)[1]*0.15):
                if xt < (np.shape(frame)[1]*0.85):
                    chyba += 1
        except:
            pass
        first = time.time(); cap.release(); time.sleep(1); # pozastavení skriptu 
        cap = cv2.VideoCapture(0);
        x,y,w,h = target(cap,lower,upper, out2);
        bbox = (x,y,w,h);
        tracker, tracker_type = createTracker(2);
        ____, frame = cap.read();
        tracker.init(frame, bbox);
        continue
    xt = int((p1[0]+p2[0])/2) # x-ová souřadnice těžiště sledovaného objektu
    yt = int((p1[1]+p2[1])/2) # y-ová souřadnice těžiště sledovaného objektu
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer); #výpočet FPS
    cv2.line(frame, (int(np.shape(frame)[1]*0.85),0), (int(np.shape(frame)[1]*0.85),int(np.shape(frame)[0])), (255,255,0), 10);  
    cv2.line(frame, (int(np.shape(frame)[1]*0.15),0), (int(np.shape(frame)[1]*0.15),int(np.shape(frame)[0])), (255,255,0), 10); #vykreslení čar započtení bodů 
    if xt > np.shape(frame)[1]*0.85:
        if state == 0:
            counter_left +=1; state = 1;     
    elif xt < np.shape(frame)[1]*0.15:
        if state == 0:
            counter_right +=1; state = 1;
    elif xt < np.shape(frame)[1]*0.75:
        if xt > np.shape(frame)[1]*0.25:
            state = 0;
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    # Zobrazení typu trackeru
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);
    # Zobrazení FPS
    cv2.putText(frame, "Chyby :" + str(chyba), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2);
    # Zobrazení neúspěšnosti
    cv2.putText(frame, str(counter_left) + " : " + str(counter_right) , (int((np.shape(frame)[1]/2)),50), cv2.FONT_HERSHEY_SIMPLEX, 2, (50,170,50),2);
    # Zobrazení skóre
    out.write(frame)        #vystup trackingu do .avi
    cv2.imshow("output", frame)
    if cv2.waitKey(2) & 0xFF == ord("q"):
        # konec programu po stisknutí klávesy "q"
        cap.release()
        break