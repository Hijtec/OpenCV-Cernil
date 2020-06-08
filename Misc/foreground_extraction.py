'''
Created on 8. 10. 2017

Manual removal of background, adapt rect to have the foreground in it
@author: Hijtec
'''
import cv2
import numpy as np
from matplotlib import pyplot as plt
from BP_scripts.functions import Release

"""
img = cv2.imread("photo.png");
mask = np.zeros(img.shape[:2], np.uint8);

bgModel = np.zeros((1,65), np.float64);
fgModel = np.zeros((1,65), np.float64);

rect = (0, 100, 400, 400); #area of foreground

cv2.grabCut(img, mask, rect, bgModel, fgModel, 5, cv2.GC_INIT_WITH_RECT);
mask2 = np.where((mask==2) | (mask==0), 0, 1).astype("uint8");
img = img*mask2[:,:,np.newaxis];

plt.imshow(img);
plt.colorbar();
plt.show()
"""

cap = cv2.VideoCapture(0);
while True:
    ___, frame = cap.read();
    mask = np.zeros(frame.shape[:2], np.uint8);
    
    bgModel = np.zeros((1,65), np.float64);
    fgModel = np.zeros((1,65), np.float64);
    
    rect = (200, 100, 400, 400);
    cv2.grabCut(frame, mask, rect, bgModel, fgModel, 5, cv2.GC_INIT_WITH_RECT); #not realtime, computationaly difficult
    mask2 = np.where((mask==2) | (mask==0), 0, 1).astype("uint8");
    frame = frame*mask2[:,:,np.newaxis];
    cv2.imshow("grabcut", frame);
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

Release()
