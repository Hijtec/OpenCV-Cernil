'''
Created on 8. 10. 2017

advanced template matching
@author: Hijtec
'''
import cv2
import numpy as np
import matplotlib.pyplot as plt

template = cv2.imread("seacity.jpg");
image = cv2.imread("tropcity.jpg");

orb = cv2.ORB_create(); #detects similarities

kp1, des1 = orb.detectAndCompute(template, None); #keypoints and descriptors for template
kp2, des2 = orb.detectAndCompute(image, None); #keypoints and descriptors for image

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

matches = bf.match(des1,des2);
matches = sorted(matches, key = lambda x:x.distance); #sorted from most likely a match to least likely a match

img3 = cv2.drawMatches(template,kp1,image,kp2, matches[:50], None, flags = 2) #what img, what to look in, another img, what to look in that one, what to show(first fifty matches), dont know, idk what flags mean
plt.imshow(img3);
plt.show();