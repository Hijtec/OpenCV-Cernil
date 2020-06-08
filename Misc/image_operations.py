import cv2
import numpy as np

img = cv2.imread("watch.jpg", cv2.IMREAD_COLOR);

px = img[55,55];
img[55,55] = [255,255,255];

img[100:150,100:150] = [255,255,0]; #region of the image

watch_face = img[37:111,107:194]; #take a region of the image
img[0:74,0:87] = watch_face; #paste it here

print(img.shape);
print(img.size);
print(img.dtype);

cv2.imshow("image", img);
cv2.waitKey(0);
cv2.destroyAllWindows();