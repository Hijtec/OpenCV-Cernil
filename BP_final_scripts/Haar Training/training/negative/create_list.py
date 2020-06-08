import cv2
import numpy as np
import glob

p = glob.glob("*.jpg")
print(p)
file = open("bg.txt", "w")
for i in p:
    file.write(str(i) + "\n")
