function [ vu, omega ] = calculateControlOutput( robotPose, goalPose, parameters )
%CALCULATECONTROLOUTPUT This function computes the motor velocities for a differential driven robot
%global count_aux;
%global ahead;
% current robot position and orientation
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% goal position and orientation
xg = goalPose(1);
yg = goalPose(2);
thetag = goalPose(3);

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);

% the following paramerters should be used:
% Task 3:
% parameters.Kalpha, parameters.Kbeta, parameters.Krho: controller tuning parameters
% Task 4:
% parameters.backwardAllowed: This boolean variable should switch the between the two controllers
% parameters.useConstantSpeed: Turn on constant speed option
% parameters.constantSpeed: The speed used when constant speed option is on
%if count_aux == 1000
%  count_aux = 0;
%endif
%if count_aux == 0
%if alpha >= -pi/2 && alpha < pi/2 
%  if normalizeAngle(-theta-thetag) < pi/2 %&& normalizeAngle(thetag-theta) >= -pi/2
%    ahead = 1;
%  else
%    ahead = 0;
%  end
%else
%  if normalizeAngle(theta-thetag) < pi/2 %&& normalizeAngle(thetag-theta) >= -pi/2
%    ahead = 0;
%  else
%    ahead = 1;
%  end
%end
%count_aux += 1;
%endif
%count_aux += 1;

%ahead = 0;
%if ahead == 1
if parameters.useConstantSpeed == true
  vu = parameters.constantSpeed;
else
  vu = parameters.Krho* rho*cos(alpha); % [m/s]
end
omega = (parameters.Kalpha*alpha + parameters.Kbeta * (normalizeAngle(normalizeAngle(-lambda)+thetag)));% [rad/s]
%else
%vu = -0.300;%parameters.Krho* rho*cos(normalizeAngle(alpha+pi)); % [m/s]
%omega = parameters.Kalpha*normalizeAngle(alpha+pi) + parameters.Kbeta * (normalizeAngle(normalizeAngle(-lambda+pi)+thetag));% [rad/s]
%end
if (alpha < -pi/2 || alpha > pi/2) && (parameters.backwardAllowed == true)
  vu = -vu;
  m = [cos(alpha) 0; -sin(alpha)/rho 1; sin(alpha)/rho 0];
  vels = m * [vu omega]';
  %printf("Negative vu: %d",vu);
  omega = vels(2)-vels(3);
  %printf("Turning around omega: %d",omega);
  %fflush(stdout);
end
