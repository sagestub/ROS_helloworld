#!/home/sage/pyenvs/ROS544proj/bin/python3

import os
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

#initialize global variables

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'

#initialize the robot node:
rospy.init_node('markerNode',anonymous=True)


#create the publisher for /pose topic:
markerPub = rospy.Publisher('/marker', Marker,queue_size=10)


def markerCallback(poseMsg):
    
    #update the marker position
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = poseMsg.x
    marker.pose.position.y = poseMsg.y
    marker.pose.position.z = poseMsg.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = poseMsg.w

    marker.scale.x = 1.0
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    markerPub.publish(marker)
    
def main():
    
    #create a subscriber to receive /cmd_vel topic messages:
    rospy.Subscriber('/pose',Quaternion,markerCallback)
        
    #keep the node going:
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
      pass