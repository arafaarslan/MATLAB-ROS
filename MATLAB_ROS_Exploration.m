%% Initialize ROS
% If no arguments are provided to rosinit, MATLAB will create both the ROS
%master and a global node
rosinit();

%%
%simu = RobotSimulator;
ExampleHelperSimulinkRobotROS('ObstacleAvoidance')

%% Get all nodes that are part of the ROS network
rosnode list


%% Get all topics that are available
rostopic list


%% Get information about /odom topic
rostopic info /odom


%% See what dfata is published on the topic
odometry = rostopic('echo','/odom')
showdetails(odometry)


%% Get information about messages published through the sensor message
rostopic info /scan
rosmsg show sensor_msgs/LaserScan


%% Create a subscriber for the /scan topic
% This uses the "rossubscriber" function
scanner = rossubscriber('scan')


%% Three Ways of Acessing Data
%% 1. Get the next message that arrives
laserdata = receive(scanner,5)
%% 2. Get the latest data that was received (might be empty)
laserdata = scanner.LatestMessage
%% 3. Set an asynchronous callback for new messages
scanner.NewMessageFcn = @(~,msg)disp(msg.Header.Seq)
%% Delete callback
scanner.NewMessageFcn = [];


%% Create a publisher to control the robot
[velcmd,vel] = rospublisher('/mobile_base/commands/velocity')


%% Drive forward and turn to the left
vel.Linear.X = 0.2;
vel.Angular.Z = 0.5;
send(velcmd,vel)


%% Stop the robot 
vel.Linear.X = 0;
vel.Angular.Z = 0;
send(velcmd,vel)


%% Drive the robot forward until it is 1 meter from the
distToWall = Inf;
while(distToWall >= 1)
	scan = scanner.LatestMessage;
	distToWall = scan.Ranges(ceil(size(scan.Ranges,1)/2))
	if isnan(distToWall)
		distToWall = Inf;
	end
	%Drive forward
	vel.Linear.X = 0.5;
	send(velcmd,vel)
	pause(0.1)
end
% Stop the robot 
vel.Linear.X = 0;
send(velcmd,vel)


%% Get all available services
rosservice list

%% Reset the simulation through a service call
% Consider moving the robot from its start location to see
% effect of the simulation reset.
[resetClient,resetMsg] = rossvcclient('/sim/reset_poses');
resetClient.call(resetMsg)


%% Close the simulator windows when you are done and shutdown
%close('Robot Simulator');
rosshutdown;
clear;
clc;
