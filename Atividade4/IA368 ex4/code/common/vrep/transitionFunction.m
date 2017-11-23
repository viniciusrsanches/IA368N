function [f, F_x, F_u] = transitionFunction(x,u, l)
% [f, F_x, F_u] = transitionFunction(x,u,l) predicts the state x at time t given
% the state at time t-1 and the input u at time t. F_x denotes the Jacobian
% of the state transition function with respect to the state evaluated at
% the state and input provided. F_u denotes the Jacobian of the state
% transition function with respect to the input evaluated at the state and
% input provided.
% State and input are defined according to "Introduction to Autonomous Mobile Robots", pp. 337

%STARTRM
pkg load symbolic
%
syms Delta_r Delta_l th X Y b
%b = l/2;
%Delta_r = u(2);
%Delta_l = u(1);
%Delta_S = ((Delta_r + Delta_l)/2);
%Delta_th= ((Delta_r - Delta_l)/l);

%X = x(1);
%Y = x(2);
%th = x(3);

f = [X Y th]'+[ (((Delta_r + Delta_l)/2)*cos(th+(((Delta_r - Delta_l)/(2*b))/2))); ...
                (((Delta_r + Delta_l)/2)*sin(th+(((Delta_r - Delta_l)/(2*b))/2))); ... 
                ((Delta_r - Delta_l)/(2*b))];


                
%Q_F_x = jacobian(f,[X Y th]);                
F_x = [ 1 0 -(Delta_S*sin(th+(Delta_th/2))); ...
        0 1 (Delta_S*cos(th+(Delta_th/2))); ...
        0 0 1];

V1 = cos(th+(Delta_th/2))/2 + (Delta_S/(2*l))*sin(th+(Delta_th/2));
V2 = cos(th+(Delta_th/2))/2 - (Delta_S/(2*l))*sin(th+(Delta_th/2));
V3 = sin(th+(Delta_th/2))/2 - (Delta_S/(2*l))*cos(th+(Delta_th/2));
V4 = sin(th+(Delta_th/2))/2 + (Delta_S/(2*l))*cos(th+(Delta_th/2));
F_u = [V1 V2; V3 V4; (-1/(l)) (1/(l))] ;
%ENDRM
