import cv2
import numpy as np
import glob

p = glob.glob("rawdata/*.png")
print(p)
file = open("info.txt", "w")
for i in p:
    gray = cv2.imread(i,0)
    rows,cols = gray.shape #získám rozlišení obrázku
    print(rows, cols)
    file.write(str(i) + " 1 3 3 " + str(cols-3) + " " + str(rows-3) + "\n")
