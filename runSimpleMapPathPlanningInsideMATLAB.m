%% Step  1 : Read a map(Binary Occupancy Grid)
    % Import image
    floorplan = imread('GazeboEnvironment.bmp');
    
    % Convert image to binary array
    grayFloorplan = rgb2gray(floorplan);
    bwFloorplan = grayFloorplan < 80.5;
    
    % Create binary occupancy grid
    map = robotics.BinaryOccupancyGrid(bwFloorplan);
    

% img = imread ('labimage.jpg');
% map = robotics.BinaryOccupancyGrid(~img,20);
map.GridLocationInWorld = [-7.03 -3.525];
figure 
show(map);
% Other option to get the map:
% Subscribe to 2-D occupancy grid from ROS mapping
% map_server package


%% Step 2 : Inflate the map with the robot size 
% Assume our robot is 0.25 meter in radius 
inflatedMap = copy(map);
inflate(inflatedMap,0.25);
figure;
show(inflatedMap);


%% Step 3 Create a (probabilistic roadmap) PRM object
% Less # of nodes : faster, but less # of possible
% More # of nodes : slower, but more # of possible
prm = robotics.PRM(inflatedMap);
prm.NumNodes = 500;
figure;
ax = axes;
show(prm,'Parent',ax);


%% Step 4 Find the path
start = [300 150]; % starting position
goal = [450 25]; % ending position
path = findpath(prm,start,goal);
hold('on');
show(prm,'Map','off','Roadmap', 'off');
hold(ax,'off');


%% Step 5 : Create PurePursuit controller to follow 
controller = robotics.PurePursuit;
controller.Waypoints = path;

% Define the start point and the goal based on the 
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
startPoint = [robotCurrentLocation,0];

%Set the path following controller parameters. The 
% velocity is set to 0.3 meters/second for this example
controller.DesiredLinearVelocity = 10;

% The maximum angular velocity acts as a saturation
% set at 2 radians/second for this example
controller.MaxAngularVelocity = 1;

% As a general rule the lookshead distance should
% linear velocity for a smooth path. The robot might
% lookshead distance is large. In contrast a small
% result in an unstable path following behavior. A
% for this example
controller.LookaheadDistance = 0.6; % determine how


%% Step 6 : Create a simulated robot for control
robot = ExampleHelperDifferentialDriveRobot(startPoint)


%% Step 7 : Show the robot running on the path
goalRadius = 10; % The robot stops when it is 
distanceToGoal = norm(robotCurrentLocation - robotGoal);
while(distanceToGoal > goalRadius)
	%compute the controller outputs, i.e. the input
	[v,omega] = step(controller, robot.CurrentPose);
	%Simulate the robot using the controller output
	drive(robot,v,omega)
	%Extract current location information (X,Y) from 
	% robot
	robotCurrentLocation = robot.CurrentPose(1:2);
	% Re-compute the distance to the goal
	distanceToGoal = norm(robotCurrentLocation - robotGoal)
end

