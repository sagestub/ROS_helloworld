#this script is for initial running of the VO algorithm:
# - it will run openCV in a loop, capturing images on keystrokes, 
# which can be used to test algorithm

import cv2 as cv
import os
import pandas as pd
import keyboard
import glob
import numpy as np
from matplotlib import pyplot as plt


filename = "capture_img_"
filetype = "jpg"
index = 0

cap = cv.VideoCapture(2) #initialize camera stream
cap.set(cv.CAP_PROP_AUTOFOCUS, 0) #disable autofocus
while cap.isOpened():
    ret, frame = cap.read() #read image values
    cv.imshow('Webcam', frame) #visualize values
    if cv.waitKey(1) & 0xFF == ord('s'): 
        cv.imwrite('{0}{1}.{2}'.format(filename,index,filetype), frame)
        print("saved image to {0}{1}.{2}".format(filename,index,filetype))
        index = index+1
    if cv.waitKey(100) & 0xFF == ord('q'): 
        break
cap.release()
cv.destroyAllWindows()