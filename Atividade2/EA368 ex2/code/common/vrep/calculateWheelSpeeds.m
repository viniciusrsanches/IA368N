function [ LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds( vu, omega, parameters )
%CALCULATEWHEELSPEEDS This function computes the motor velocities for a differential driven robot

wheelRadius = parameters.wheelRadius;
halfWheelbase = parameters.interWheelDistance/2;

M = [parameters.wheelRadius/2 parameters.wheelRadius/2; parameters.wheelRadius/(parameters.interWheelDistance) -parameters.wheelRadius/(parameters.interWheelDistance)];
v = [vu; omega];
wheels_velocity = M \ v;
LeftWheelVelocity = wheels_velocity(2);
RightWheelVelocity = wheels_velocity(1);

end
