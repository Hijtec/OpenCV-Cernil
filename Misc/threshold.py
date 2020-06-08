'''
Created on 6. 10. 2017

@author: Hijtec
'''
import cv2
import numpy as np

img = cv2.imread("bookpage.jpg");
grayscaled = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY);
retval, threshold = cv2.threshold(grayscaled, 10, 255, cv2.THRESH_BINARY); # trying thresholding a grayscale img, pixely s value více jak 10 se stanou 255

adaptivethreshold = cv2.adaptiveThreshold(grayscaled, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1) #adaptivn9 threshold
otsuthreshold = cv2.threshold(grayscaled, 125, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) #a ještě jedna varianta

cv2.imshow("original", img);
cv2.imshow("threshold", threshold);
cv2.imshow("adaptivethreshold", adaptivethreshold);
cv2.imshow("otsuthreshold", adaptivethreshold);

cv2.waitKey(0);
cv2.destroyAllWindows();
