#!/home/sage/pyenvs/ROS544proj/bin/python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Header

lastTime = None
posePub = None

#initialize the robot node:
rospy.init_node('robotNode',anonymous=True)

#create the publisher for /pose topic:
posePub = rospy.Publisher('/pose', Pose2D,queue_size=10)

def robotPoseCallback(twistMsg):
    global posePub
    #calculate time elapsed from last 
    global lastTime
    currentTime = rospy.Time.now()
    if lastTime is not None:
        dt = (currentTime-lastTime).to_sec()

        #use received twist message to update position values:
        pose.x += twistMsg.linear.x*dt
        pose.y += twistMsg.linear.y*dt
        pose.theta += twistMsg.angular.z*dt

        pose_msg = Pose2D()
        pose_msg.x = pose.x
        pose_msg.y = pose.y
        pose_msg.theta = pose.theta

        #publish the updated pose
        posePub.publish(pose_msg)
    #update lastTime for next loop:
    lastTime = currentTime
    
def main():
    #create a subscriber to receive /cmd_vel topic messages:
    rospy.Subscriber('/cmd_vel',Twist,robotPoseCallback)

    #give initial pose values:
    pose = Pose2D()
    pose.x = 0.0
    pose.y = 0.0
    pose.theta = 0.0
    posePub.publish(pose)

    #start the node:
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass