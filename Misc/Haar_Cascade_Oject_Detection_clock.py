'''
Created on 9. 10. 2017
Using Haar Cascades to detect various objects
@author: Hijtec
'''
import cv2
import numpy as np

clock = cv2.CascadeClassifier("haarcascades/carkeys.xml");
eye_cascade = cv2.CascadeClassifier("haarcascades/haarcascade_eye.xml");
eyeglasses_cascade = cv2.CascadeClassifier("haarcascades/haarcascade_eye_tree_eyeglasses.xml");
smile_cascade = cv2.CascadeClassifier("haarcascades/haarcascade_smile.xml");
body_cascade = cv2.CascadeClassifier("haarcascades/haarcascade_fullbody.xml");
profile_cascade = cv2.CascadeClassifier("haarcascades/haarcascade_profileface.xml");

cap = cv2.VideoCapture(0);
#fourcc = cv2.VideoWriter_fourcc(*"XVID");
#out = cv2.VideoWriter("output.avi",fourcc,28.0,(640,480));

while(True):
    ret, img = cap.read();
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY);
    faces = clock.detectMultiScale(gray);
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2);
        roi_gray = gray[y:y+h, x:x+w];
        roi_color = img[y:y+h, x:x+w];
        eyes = eye_cascade.detectMultiScale(roi_gray);
        smile = smile_cascade.detectMultiScale(roi_gray, 1.3, 25);
        eyeglasses = eyeglasses_cascade.detectMultiScale(roi_gray);
        
           
        for (sx,sy,sw,sh) in smile:
            cv2.rectangle(roi_color, (sx,sy), (sx+sw, sy+sh), (0,0,255), 2);
        
        for (gx,gy,gw,gh) in eyeglasses:
            cv2.rectangle(roi_color, (gx,gy), (gx+gw, gy+gh), (0,255,255), 2);
    
    cv2.imshow("img", img)
    #out.write(img);
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release();
cv2.destroyAllWindows();