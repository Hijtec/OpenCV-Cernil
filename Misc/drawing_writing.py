import numpy as np
import cv2

cap = cv2.imread("watch.jpg", cv2.IMREAD_COLOR);
cv2.line(cap, (0,0), (150,150), (255,255,255), 10); #where, start, end, color, width
cv2.rectangle(cap, (25,10), (150,80), (255,0,0), 5); #where, start, end, color, width
cv2.circle(cap, (150,150), 55, (0,0,255), 2); #where, middle, radius, color, width (-1 equals to filled circle)

pts = np.array([[10,50],[20,300],[700,20],[50,100]], np.int32);
cv2.polylines(cap, [pts], True, (0,255,0), 5); #where, what to draw, join first with last?, color, width

font = cv2.FONT_HERSHEY_SIMPLEX; #fonty
cv2.putText(cap, "OpenCV RULEZ", (0,130), font, 1, (255,255,0),2 ,cv2.LINE_AA); #where, what, start, type, size, color, thickness, antialiasing


cv2.imshow("image", cap);
cv2.waitKey(0);
cv2.destroyAllWindows()
