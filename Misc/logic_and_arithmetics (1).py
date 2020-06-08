import cv2
import numpy as np

img1 = cv2.imread("seacity.jpg");
img2 = cv2.imread("tropcity.jpg");
img3 = cv2.imread("logo.jpg");

#add = img1 + img2; add pixelcolors together
#add = cv2.add(img1,img2); add pixelcolors together (limited to 255,255,255)
#weighted = cv2.addWeighted(img1, 0.6, img2, 0.4, 0) #add together with weight, last 0 is gamma parameter
#cv2.imshow("add",weighted);

rows, cols, channels = img3.shape; #gets our img3 size and colors parameters
roi = img1[0:rows,0:cols]; #defining region of image through the up-mentioned parameters, basically take all img3

img2gray = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY);
ret, mask = cv2.threshold(img2gray,150, 255, cv2.THRESH_BINARY_INV); #if a pixel is above 220, it will be converted to 255(white), if below 220, it becomes 0(black)

mask_inv = cv2.bitwise_not(mask);#logical operations not, and
img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv); # Now black-out the area of logo in ROI
img3_fg = cv2.bitwise_and(img3, img3, mask=mask); # Take only region of logo from logo image.

collect = cv2.add(img1_bg, img3_fg);
img1[0:rows, 0:cols] = collect;



cv2.imshow("result", img1)

cv2.waitKey(0);
cv2.destroyAllWindows();