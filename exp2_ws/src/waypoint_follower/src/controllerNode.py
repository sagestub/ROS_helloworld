#!/home/sage/pyenvs/ROS544proj/bin/python3

import os
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from nav_msgs.msg import Path

#specify ros master
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.33:11311'

#initialize the controller node:
rospy.init_node('controllerNode',anonymous=True)
print("controllerNode: Initialized")

#create the publisher for /cmd_vel topic:
twistPub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
print("controllerNode: created /cmd_vel publisher")

#create the publisher for /path topic:
pathPub = rospy.Publisher('/path', Path,queue_size=24)

#create the publisher for /goal topic:
cmd_goalPub = rospy.Publisher('/cmd_goal', Pose2D ,queue_size=10)

#create global variables:
cmd_vel = None
goal = 0
# waypoints = np.array([[4.63202999999382,-2.80608000000182],[4.22687999999141,-0.530580000001208],\
#         [6.53789999999304,0.983460000001415],[7.07402999999601,1.89032999999903],\
#         [8.01420000000419,2.35320000000161],[8.37162000000092,3.39437999999927],\
#         [9.32399999999404,3.88832999999831],[11.0511600000028,3.37329000000125],\
#         [12.5163599999946,4.50659999999896],[13.337760000003,3.85724999999791],\
#         [15.6720899999931,4.32123000000164],[18.31610999999,7.43145000000062],\
#         [4.96614000000207,5.13485999999805],[6.25151999999048,3.27006000000161],\
#         [5.18036999999751,3.22898999999882],[4.3345500000035,4.26905999999928],\
#         [3.61970999999428,4.30013999999968],[4.70418000000095,2.41535999999847],\
#         [3.87056999999189,1.23098999999954],[2.0834699999925,1.34421000000156],\
#         [0.451769999995122,3.11577000000074],[-2.67954000000188,1.97247000000065],[0,0]])

waypoints = np.array([[0,-1],[-1,-1],[-1,1],[1,1],[1,-1]])

#publish goal pose
path_msg = Path()
path_msg.header.frame_id = 'map'

# for waypoint in waypoints: 
#     waypoint_msg = PoseStamped()
#     waypoint_msg.header.frame_id = 'map'
#     waypoint_msg.pose.position.x = waypoint[0]
#     waypoint_msg.pose.position.y = waypoint[1]
#     waypoint_msg.pose.position.z = 0.0
#     waypoint_msg.pose.orientation.w = 1

#     path_msg.poses.append(waypoint_msg)

# pathPub.publish(path_msg)

def cmd_velControllerCallback(poseMsg):
    # print("controllerNode: callback updating /cmd_vel")
    global twistPub
    global pathPub
    global cmd_goalPub
    global cmd_vel
    global goal
    global waypoints
    kp = 0.5
    ka = 3
    vmax = 1.5
    wmax = 1
    #use received pose message to update position values:
    dx = waypoints[goal,0]-poseMsg.x
    dy = waypoints[goal,1]-poseMsg.y
    
    #calculate position and heading error
    rho = np.sqrt(dx**2+dy**2)
    alpha = -poseMsg.theta+np.arctan2(dy,dx)

    #publish goal message
    goal_msg = Pose2D()
    goal_msg.x = waypoints[goal,0]
    goal_msg.y = waypoints[goal,1]
    goal_msg.theta = alpha
    cmd_goalPub.publish(goal_msg)

    #determine control goal and state:
    if rho < 0.1: #if close enough to current waypoint, start going next waypoint
        if goal == len(waypoints)-1:
            goal =0 #start over
        else:
            goal += 1
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = 'map'
        waypoint_msg.pose.position.x = waypoints[goal,0]
        waypoint_msg.pose.position.y = waypoints[goal,1]
        waypoint_msg.pose.position.z = 0.0
        waypoint_msg.pose.orientation.w = 1
        path_msg.poses.append(waypoint_msg)
        pathPub.publish(path_msg)
        print('New Goal: {0}, {1}'.format(waypoints[goal,0],waypoints[goal,1]))
        
    if np.abs(alpha)> np.pi/12: #if robot not aimed towards waypoint
        v = 0 #linear speed = 0
        w = ka*alpha #control robot to aim at waypoint
    else:
        v = min(abs(kp*rho),vmax)*np.sign(kp*rho) #vmax #
        w = min(ka*alpha,wmax)*np.sign(ka*alpha) #ka*alpha #
    # v = kp
    # w = ka*alpha

    twist_msg = Twist()
    twist_msg.linear.x = v*np.cos(poseMsg.theta)
    twist_msg.linear.y = v*np.sin(poseMsg.theta)
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = w

    #publish the updated pose
    twistPub.publish(twist_msg)
    # print("controllerNode: published /cmd_vel")
    
def main():
    global twistPub
    global cmd_vel
    global pathPub
    global goalPub

    # while not rospy.is_shutdown():
    # print("controllerNode: creating /pose subscriber")
    #create a subscriber to receive /pose topic messages:
    rospy.Subscriber('/pose',Pose2D,cmd_velControllerCallback)

    #keep the node going:
    rospy.spin()
if __name__ == '__main__':
    main()