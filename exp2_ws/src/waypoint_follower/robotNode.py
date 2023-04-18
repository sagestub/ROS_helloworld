#!/home/sage/pyenvs/ROS544proj/bin/python3

import os
import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Header

#initialize global variables
lastTime = None
pose_msg= None

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'

#initialize the robot node:
rospy.init_node('robotNode',anonymous=True)
print("initialized robot_node")

#create the publisher for /pose topic:
posePub = rospy.Publisher('/pose', Pose2D,queue_size=10)
print("initialized robot_node publisher")

def robotPoseCallback(twistMsg):
    print("executing robotPoseCallback")
    global posePub
    global pose_msg

    
    #calculate time elapsed from last 
    global lastTime
    currentTime = rospy.Time.now()
    if lastTime is not None:
        dt = (currentTime-lastTime).to_sec()

        #use received twist message to update position values:
        pose_msg.x += twistMsg.linear.x*dt
        pose_msg.y += twistMsg.linear.y*dt
        pose_msg.theta += twistMsg.angular.z*dt

        #publish the updated pose
        posePub.publish(pose_msg)
        print("published pose")
    #update lastTime for next loop:
    lastTime = currentTime
    
def main():
    global posePub
    global pose_msg
    print("executing robot_node main method")
    
    if pose_msg is None: #if first time executing
        #give initial pose values:
        pose_msg = Pose2D()
        pose_msg.x = 0.0
        pose_msg.y = 0.0
        pose_msg.theta = 0.0
        print(pose_msg)
        posePub.publish(pose_msg)
        print("published initial pose")

    else:
        #create a subscriber to receive /cmd_vel topic messages:
        rospy.Subscriber('/cmd_vel',Twist,robotPoseCallback)
    
    #keep the node going:
    rospy.spin()
if __name__ == '__main__':
    main()
#    except rospy.ROSInterruptException:
#       pass