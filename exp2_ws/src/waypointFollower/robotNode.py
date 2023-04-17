#!/pyenvs/ROS544proj/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

#initialize the ROS node:
rospy.init_node('robotNode',anonymous=True)

#Create the publisher for /pose topic:
posePub = rospy.Publisher('/pose', Pose2D)


def publisher():
    posePub = rospy.Publisher('/pose',Pose2D)
    rospy.init_node('robotNode',anonymous=True)
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        #do stuff
        #create pose message object
        pose_msg = Pose2D()
        #set pose values:
        pose_msg.x = 1.0;
        pose_msg.y = 2.0;
        pose_msg.theta = 0.0;
        posePub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass