%% V-REP Simulation Exercise 3: Kinematic Control
% Tests the implemented control algorithm within a V-Rep simulation.

% In order to run the simulation:
%   - Start V-Rep
%   - Load the scene matlab/common/vrep/mooc_exercise.ttt
%   - Hit the run button
%   - Start this script


clear;
close all; 
%% Parameters setup

%% define that we will use the real P3DX and/or the simulated one
global realRobot ; 
realRobot=1;

%global ghostPose;
%ghostPose=[0,0,0];

global laserStr;
laserStr = '/perception/laser/';

global laserIndex;
laserIndex='1';

global optionStr;
optionStr= '?range=-90:90:10'; %  example optionStr= '?range=-90:90:3' or optionStr=''; if no options are required

global poseStr;
poseStr = '/motion/pose';   

global vel2Str;
vel2Str = '/motion/vel2';   

global stopStr;
stopStr = '/motion/stop';

global parameters;
%% initialization
 connection = simulation_setup();

if realRobot==1
    % http_init will add the necessary paths 
    %http_init('SID_7755');
    http_init();
    % Declaration of variables
    %connection = 'http://10.1.3.130:4950';  %use this address if you are
    %connected locally to the robot in the REALabs wifi network
    %connection = 'http://143.106.148.171:9090/resource/RobotFEEC2';
    connection = 'http://192.168.0.105:4950';
    parameters.wheelDiameter = .195;
    parameters.wheelRadius = parameters.wheelDiameter/2.0;
    parameters.interWheelDistance = .381/2;
    
    %1% Pioneer_p3dx_setTargetGhostPose(connection, -2, 0, 0);
    
    %% Set the initial pose of the robot ( TO CHANGE INTO A FUNCTION SET_POSE)
    %1%http_put([connection '/motion/pose'], struct ('x',0,'y',-2 *1000,'th',90)) ;
    %http_put([connection '/motion/pose'], struct ('x',0,'y',0,'th',0)) ;
    
   % parameters.scannerPoseWrtPioneer_p3dx = Pioneer_p3dx_getScannerPose(connection);
else 
    %% Initialize connection with V-Rep
    connection = simulation_openConnection(connection, 0);
    simulation_start(connection);

    %% Get static data from V-Rep
    Pioneer_p3dx_init(connection);

end

pause(1)


%% reading laser
%%%PUT YOUR CODE HERE
%dist = [];
%count = 0;
%while count < 1000
%  aux = [];
%  aux = Pioneer_p3dx_getLaserData(connection,'local_poses');
%  %if ~ismember(aux,dist,"rows")
%    dist = [dist; aux];
%  %endif
%  count ++;
%  %pause(0.250);
%endwhile

dist = csvread('./data.txt');

rows = size(dist)(1);
colums = size(dist)(2);
deviation = [];
average = [];
variance = [];
precision = [];
for c=1:colums
  deviation= [deviation std(dist(1:100,c))];
  average = [average mean(dist(1:100,c))];
  variance = [variance var(dist(1:100,c))];
  precision = [precision ((max(dist(1:100,c))-min(dist(1:100,c)))^2/deviation(c))];
  [H, pValue, W] = swtest(dist(1:100,c),0.01);
  if H == 0
    figure(c);
    hist(dist(:,c),30);
  end
  printf ("H: %d\n",H);
  printf("pValue: %d\n",pValue);
  printf("W: %d\n",W);
  fflush(stdout);
endfor



if realRobot~= 1
     simulation_stop(connection);
     simulation_closeConnection(connection);
% else
     
 end
% msgbox('Simulation ended');

