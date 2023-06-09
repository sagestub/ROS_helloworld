#! /usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Pose2D, Twist

#initialize global variables
pose_msg= None
lastTime = None

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'

#initialize the robot node:
rospy.init_node('robotNode',anonymous=True)
print("robotNode: Initialized")

#create the publisher for /pose topic:
posePub = rospy.Publisher('/pose', Pose2D,queue_size=10)
print("robotNode: created /pose publisher")

def robotPoseCallback(twistMsg):
    # print("robotNode: callback updating pose")
    global posePub
    global pose_msg
    global lastTime
    currentTime=rospy.Time.now()

    #calculate updated position
    if pose_msg is None: #if first time executing
        #give initial pose values:
        pose_msg = Pose2D(x=0.0, y=0.0, theta=0.0)
        dt = 0.1 #dummy initial timestep
    else:
        if lastTime is not None:
            dt=(currentTime-lastTime).to_sec()
        else: dt=0.1
        #use received twist message to update position values:
        pose_msg.x += twistMsg.linear.x*dt
        pose_msg.y += twistMsg.linear.y*dt
        pose_msg.theta += twistMsg.angular.z*dt
        lastTime= currentTime

    #publish the updated pose
    posePub.publish(pose_msg)
    # print("robotNode: published /pose")
    
def main():
    global posePub
    global pose_msg

    #create a subscriber to receive /cmd_vel topic messages:
    rospy.Subscriber('/cmd_vel',Twist,robotPoseCallback)
        
    #keep the node going:
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
      pass