%% Start ROS
clear all
clf
%% start ros session with matlab node connected to local network master
masterURI ='http://192.168.1.33:11311' 
% localURI = 'http://192.168.1.9';

rosinit(masterURI)
%% Initialize the node running VO algorithm on Matlab
% nodeName = rosgenmsg('MatlabNode');
% masterHost = 'http://192.168.1.33:11311';
suffix = num2str(randi([0,1000]));
nodeName = ['vo_node','_',suffix]

vo_node = robotics.ros.Node(nodeName);

%% Initialize publisher and subscriber
%create global handle for callback fuction to use for
%reading and publishing values
global rosmsg_handles 
rosmsg_handles.imgSub = robotics.ros.Subscriber(vo_node,'/img',@voCallbackFn);

rosmsg_handles.posePub = robotics.ros.Publisher(vo_node,'/pose','geometry_msgs/Pose');
rosmsg_handles.posePubmsg = rosmessage(rosmsg_handles.posePub);

rosmsg_handles.markerPub = robotics.ros.Publisher(vo_node,'/marker','visualization_msgs/Marker');
rosmsg_handles.markerPubmsg = rosmessage(rosmsg_handles.markerPub);
