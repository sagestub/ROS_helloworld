#!/pyenvs/ROS544proj/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist

#initialize the controller node:
rospy.init_node('controllerNode',anonymous=True)

#create the publisher for /cmd_vel topic:
twistPub = rospy.Publisher('/cmd_vel', Pose2D)

#create global variables:
cmd_vel = None
goal = None
waypoints = np.array([[0,0],[4.63202999999382,-2.80608000000182],[4.22687999999141,-0.530580000001208],\
        [6.53789999999304,0.983460000001415],[7.07402999999601,1.89032999999903],\
        [8.01420000000419,2.35320000000161],[8.37162000000092,3.39437999999927],\
        [9.32399999999404,3.88832999999831],[11.0511600000028,3.37329000000125],\
        [12.5163599999946,4.50659999999896],[13.337760000003,3.85724999999791],\
        [15.6720899999931,4.32123000000164],[18.31610999999,7.43145000000062],\
        [4.96614000000207,5.13485999999805],[6.25151999999048,3.27006000000161],\
        [5.18036999999751,3.22898999999882],[4.3345500000035,4.26905999999928],\
        [3.61970999999428,4.30013999999968],[4.70418000000095,2.41535999999847],\
        [3.87056999999189,1.23098999999954],[2.0834699999925,1.34421000000156],\
        [0.451769999995122,3.11577000000074],[-2.67954000000188,1.97247000000065],[0,0]])

def cmd_velControllerCallback(poseMsg):
    global twistPub
    global cmd_vel
    global goal
    global waypoints
    #use received pose message to update position values:
    dx = waypoints[goal,0]-poseMsg.X
    dy = waypoints[goal,1]-poseMsg.Y

    #calculate position and heading error
    rho = np.sqrt(dx^2+dy^2)
    alpha = -poseMsg.Theta+np.atan2(dy,dx)

    #determine control goal and state:
    if rho < 0.1: #if close enough to current waypoint, start going next waypoint
        if goal == len(waypoints)/2:
            goal =1 #start over
        else:
            goal += 1

    if np.abs(alpha)> np.pi/12: #if robot not aimed towards waypoint
        v = 0 #linear speed = 0
        w = ka*alpha #control robot to aim at waypoint
    else:
        v = vmax #min(abs(kp*rho),vmax)*np.sign(kp*rho)
        w = ka*alpha #min(ka*alpha,wmax)*np.sign(ka*apha)

    twist_msg = Twist()
    twist_msg.linear.x = v*np.cos(poseMsg.Theta)
    twist_msg.linear.y = v*np.sin(poseMsg.Theta)
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = w

    #publish the updated pose
    twistPub.publish(twist_msg)
    
def main():
    global twistPub
    global cmd_vel
    #create a subscriber to receive /pose topic messages:
    rospy.Subscriber('/pose',Twist,cmd_velControllerCallback)

    #give initial command values:
    if cmd_vel is None:
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0

    #start the node:
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass