#! /usr/bin/env python3

import os
import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'
user = os.environ['USER']
if user == "sage":
    usb_chan = 2
    intrinsics_dir = "sage/Documents/ROS_helloworld"
if user == "orangepi":
    usb_chan = 1
    intrinsics_dir = "orangepi/ROS_helloworld"
else:
    usb_chan = 2 #guess at default value?

#how to load camera intrinsics from a file:
def load_intrinsics(dir,fileheader="lifecam"):
    mtx =  np.genfromtxt("{0}{1}_mtx.csv".format(dir,fileheader),delimiter = ",")
    dist = np.genfromtxt("{0}{1}_dist.csv".format(dir,fileheader),delimiter = ",")
    return mtx, dist
    
def publishImages():
    #initialize the robot node:
    rospy.init_node('CameraNode',anonymous=True)
    print("cameraNode: Initialized")

    #create the publisher for /pose topic:
    imgPub = rospy.Publisher('/img', Image,queue_size=10)
    print("cameraNode: created /img publisher")

    rate = rospy.Rate(10) #5hz
    cap = cv.VideoCapture(usb_chan)
    cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
    mtx, dist = load_intrinsics("/home/{}/VO_ws/".format(intrinsics_dir))
    while not rospy.is_shutdown() & cap.isOpened():
        
        ret, frame = cap.read()
        h, w = frame.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # undistort
        undst = cv.undistort(frame, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        frame = undst[y:y+h, x:x+w]
        bridge = CvBridge()
        img_msg = Image()
        img_msg.encoding = "bgr8"
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "camera_frame"
        img_msg.height = h
        img_msg.width = w
        img_msg.step = w*3
        img_msg.data = bridge.cv2_to_imgmsg(frame,"bgr8").data
        imgPub.publish(img_msg)
        rate.sleep()
        

    # Releases the webcam
    cap.release()
    # Closes the frame
    cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        publishImages()
    except rospy.ROSInterruptException:
      pass