%% Load the file
% robotData = rosbag([RSTWebinar_Dir '\Data\Rosbags' '\robot_circle.bag'])
robotData = rosbag('2011-01-24-06-18-27.bag');

%% See all topics stored in the rosbag
robotData.AvailableTopics


%% Select all messages on the odometry topic
odom = select(robotData, 'Topic', '/base_odometry/odom')

%% Get the first odometry message and show details of its content
odomsg = readMessages(odom,1)
odomsg{1}.showdetails


%% Extract the robots X and Y position as a timeseries
ts = timeseries(odom,'Pose.Pose.Position.X','Pose.Pose.Position.Y')

%% Plot the time series
figure;
plot(ts)

%% Get all the depth camera points
depthPoints = select(robotData,'Topic','/camera/depth/points')

%% Get one of the depth camera messages
ptcloud = readMessages(depthPoints,30)
ptcloud = ptcloud{1}

%% Display the point cloud in a figure window
figure;
scatter3(ptcloud,'MarkerEdgeColor','blue')


%% Get all point cloudss recorded in a 6 second segment
timedDepthPoints = select(depthPoints,'Time',[depthPoints.StartTime+3, ...
	depthPoints.StartTime+9])
%% Get all the point clouds
allPointClouds = readMessage(timedDepthPoints);

%% Display the point cloud in an animation in the CVST
figure;
for i=1:size(allPointClouds)
	ptCloudXYZ = allPointClouds(i).readXYZ;
	showPointCloud(ptCloudXYZ,'VerticalAxis','Y', ...
		'VerticalAxisDir','Down');
	drawNow;
end


%% Close all windows and clear the workspace when you are done
close('all')
clear
