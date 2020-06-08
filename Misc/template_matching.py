'''
Created on 8. 10. 2017

@author: Hijtec
'''

import cv2
import numpy as np
import glob
images = [];
images_gray = [];
for img in glob.glob("template*.jpg"):
    foundimg = cv2.imread(img);
    images.append(foundimg)
    foundimg_gray = cv2.cvtColor(foundimg, cv2.COLOR_BGR2GRAY);
    images_gray.append(foundimg_gray);

templates = [];
for temp in glob.glob("piskot*.jpg"):
    n = cv2.imread(temp, 0);
    templates.append(n);

i = 0;
colors = [(255,255,255),(0,255,255),(255,0,255),(255,255,0),(0,0,0), (125,125,125)]
for img in images_gray:
    
    for temp in templates:
        res = cv2.matchTemplate(img, temp, cv2.TM_CCOEFF_NORMED);
        w, h = temp.shape[::-1];
        
        threshold = 0.72;
        loc = np.where(res >= threshold);
        for pt in zip(*loc[::-1]):
            cv2.rectangle(images[i], pt, (pt[0]+w, pt[1]+h), colors[i], 1);
    i +=1
    
"""img_bgr = cv2.imread("template1.jpg");
img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY);
template = cv2.imread("piskot1.jpg", 0);
    
w, h = template.shape[::-1];#gets width and height

res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED);    
threshold = 0.7;
loc = np.where( res >= threshold);

for pt in zip(*loc[::-1]):
    cv2.rectangle(img_bgr, pt, (pt[0]+w, pt[1]+h), (0,255,255), 1);"""

i = 1;
for img in images:
    cv2.namedWindow("detected" + str(i),cv2.WINDOW_NORMAL);
    cv2.resizeWindow("detected", 1080, 1080);
    cv2.imshow("detected" + str(i), img);
    i +=1;
cv2.waitKey(0)  
cv2.destroyAllWindows();