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
intrinsics_dir = "/home/sage/Documents/ROS_helloworld/VO_ws/"

#load camera intrinsics - argument points to folder containing the .csv files
def load_intrinsics(dir,fileheader="lifecam"):
    mtx =  np.genfromtxt("{0}{1}_mtx.csv".format(dir,fileheader),delimiter = ",")
    dist = np.genfromtxt("{0}{1}_dist.csv".format(dir,fileheader),delimiter = ",")
    return mtx, dist

mtx, dist = load_intrinsics(intrinsics_dir)

cap = cv.VideoCapture(2) #initialize camera stream
cap.set(cv.CAP_PROP_AUTOFOCUS, 0) #disable autofocus
while cap.isOpened():
    ret, frame = cap.read() #read image values

    #undistort image
    h, w = frame.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    undst = cv.undistort(frame, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    undst = undst[y:y+h, x:x+w]
    frame = undst

    cv.imshow('Webcam', frame) #visualize values
    if cv.waitKey(20) & 0xFF == ord('s'): 
        cv.imwrite('{0}{1}.{2}'.format(filename,index,filetype), frame)
        print("saved image to {0}{1}.{2}".format(filename,index,filetype))
        index = index+1
    if cv.waitKey(10) & 0xFF == ord('q'): 
        break
cap.release()
cv.destroyAllWindows()