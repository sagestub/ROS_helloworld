#!/usr/bin/env $PYENV_ROS544PROJ

import os
import glob
import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'

#how to load camera intrinsics from a file:
def load_intrinsics(dir,fileheader="lifecam"):
    mtx =  np.genfromtxt("{0}{1}_mtx.csv".format(dir,fileheader),delimiter = ",")
    dist = np.genfromtxt("{0}{1}_dist.csv".format(dir,fileheader),delimiter = ",")
    return mtx, dist
    
def publishImages():
    #initialize the robot node:
    rospy.init_node('imageNode',anonymous=True)
    print("imageNode: Initialized")

    #create the publisher for /pose topic:
    imgPub = rospy.Publisher('/img', Image,queue_size=10)
    print("imageNode: created /img publisher")

    rate = rospy.Rate(10) #5hz
    files = sorted(glob.glob("/home/sage/Documents/ROS_helloworld/VO_ws/images/*.jpg"))
    index = 0
    max = len(files)
    while not rospy.is_shutdown() & cap.isOpened() & index<max:
        cap = cv.imread(files[index])
        ret, frame = cap.read()
        h, w = frame.shape[:2]
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