import numpy as np
import cv2

cap = cv2.VideoCapture(0)


while (True):
    ret, frame = cap.read();
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY);
        cv2.imshow('frame', gray);
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

def Session():
    return True
    if cv2.waitKey(1) & 0xFF == ord("q"):
        return None
    
def Release(cap,cap2):
    try:
        cap.release();
    except:
        pass
    try:
        cap2.release();
    except:
        pass
    cv2.destroyAllWindows();