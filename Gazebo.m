%% Initialize ROS
ipaddress = 'http://192.168.248.130:11311/';
rosinit(ipaddress);

%% Get all topics that are available
rostopic list

%% Create a publisher to control the robot
robot = rospublisher('/cmd_vel'); 
velmsg = rosmessage(robot);


%% Create a subscriber for the /scan topic
laser= rossubscriber('/scan');


%%
rosservice list

%% Reset the simulation through a service call
% Consider moving the robot from its start location to see
% effect of the simulation reset.
[resetClient,resetMsg] = rossvcclient('/gazebo/reset_simulation');
resetClient.call(resetMsg)

%% Drive the robot forward until it is 1 meter from the
distToWall = Inf;
while(distToWall >= 1)
	scan = laser.LatestMessage;
	distToWall = scan.Ranges(ceil(size(scan.Ranges,1)/2))
	if isnan(distToWall)
		distToWall = Inf;
	end
	%Drive forward
	velmsg.Linear.X = -0.5;
	send(robot,velmsg)
	pause(1)
end
% Stop the robot 
velmsg.Linear.X = 0;
send(robot,velmsg)

%% 
velmsg.Linear.X = 0;% meters per second
send(robot,velmsg)
pause(4)
velMsg.Linear.X = 0;
send(robot,velmsg)

%%
%stop(wanderHelper);
rosshutdown
