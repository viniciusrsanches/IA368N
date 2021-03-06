clc; clear all;
pkg load symbolic;
syms alpha beta gamma real

q = [alpha;beta;gamma];

r_BF_inB = [...
    - sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];


% determine the foot point Jacobian J_BF_inB=d(r_BF_inB)/dq
J_BF_inB = [0,- cos(beta + gamma) - cos(beta),-cos(beta + gamma);...
            cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
            sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),sin(beta + gamma)*cos(alpha)];

% what generalized velocity dq do you have to apply in a configuration q = [0;60�;-120�]
% to lift the foot in vertical direction with v = [0;0;-1m/s];
v = [0; 0; -1];
qi = [0; 60*(pi/180); -120*(pi/180)];

% Determine the numerical value of the foot point jacobian for initial joint angles qi
JBF = double(subs(J_BF_inB,[alpha beta gamma],qi'));
pseudo_inverted_jacobian =  inv(JBF'*JBF)*JBF';
% Determine the numerical value for dq
dq = pseudo_inverted_jacobian*v;

valid