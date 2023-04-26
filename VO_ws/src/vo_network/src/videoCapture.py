#this script is for initial running of the VO algorithm:
# - it will save a video file, then convert the video to undistorted .jpg images

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
framerate = 20.0
height = 480
width = 640


#load camera intrinsics - argument points to folder containing the .csv files
def load_intrinsics(dir,fileheader="lifecam"):
    mtx =  np.genfromtxt("{0}{1}_mtx.csv".format(dir,fileheader),delimiter = ",")
    dist = np.genfromtxt("{0}{1}_dist.csv".format(dir,fileheader),delimiter = ",")
    return mtx, dist

#method for capturing a video file in .mp4 format
def record_video(filename,fps,h,w):
    cap = cv.VideoCapture(2)
    cap.set(cv.CAP_PROP_AUTOFOCUS, 0) #disable autofocus

    # Define the codec and create a VideoWriter object
    fourcc = cv.VideoWriter_fourcc(*'mp4v') # codec for mp4 format
    out = cv.VideoWriter('{}.mp4'.format(filename), fourcc, fps, (w,h))

    # Start recording
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            # Write the frame to the output file
            out.write(frame)

            # Display the resulting frame
            cv.imshow('frame',frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    # Release everything when done
    cap.release()
    out.release()
    cv.destroyAllWindows()

mtx, dist = load_intrinsics(intrinsics_dir)

record_video("demo_01",framerate,height,width)

cap = cv.VideoCapture("demo_01.mp4")

frame_count = 0
while cap.isOpened():
    ret, frame = cap.read() #read image values
    if not ret:
        break
    #undistort image
    h, w = frame.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    undst = cv.undistort(frame, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    undst = undst[y:y+h, x:x+w]
    frame = undst

    cv.imwrite("{}{:05d}.{}".format(filename,frame_count,filetype), frame)
    frame_count += 1
cap.release()
cv.destroyAllWindows()