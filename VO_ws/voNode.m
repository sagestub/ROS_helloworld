%% Start ROS
clear all
clf
rosinit
%% Initialize the node running VO algorithm on Matlab
masterHost = 'http://192.168.1.33:11311';
vo_node = robotics.ros.Node('vo_node', masterHost);

%% Initialize publisher and subscriber
%create global handle for callback fuction to use for
%reading and publishing values
global rosmsg_handles 
rosmsg_handles.imgSub = robotics.ros.Subscriber(vo_node,'/img',@voCallbackFn);

rosmsg_handles.posePub = robotics.ros.Publisher(vo_node,'/pose','geometry_msgs/Pose');
rosmsg_handles.posePubmsg = rosmessage(rosmsg_handles.posePub);
