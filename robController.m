function [ tau ] = robController( trajectory, Theta, Theta_dot, t , rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function sends command torques in the vector tau to move the
%    robot. The controller determines tau based on the desired position and
%    velocity and the actual position and velocity. Desired position and
%    velocity are obtained from trajectory and the current time t. The
%    actual position is Theta, and the actual velocity is Theta_dot.
%    Calculate the torques needed to add gravity compensation using the
%    robot parameters from the structure rob.
%

% Robot Parameters from rob
g = rob.parameters.g;
b = rob.parameters.b;
m1 = rob.parameters.m1;
m2 = rob.parameters.m2;
m3 = rob.parameters.m3;
m4 = rob.parameters.m4;
l1 = rob.parameters.l1;
l2 = rob.parameters.l2;
l3 = rob.parameters.l3;
I1 = rob.parameters.I1;
I2 = rob.parameters.I2;
J1 = rob.parameters.J1;
J2 = rob.parameters.J2;
J3 = rob.parameters.J3;
    
% Gravity Compensation Vector
G = [m1*g*cos(Theta(1)); m2*g*cos(Theta(2)); (m3+m4)*g*cos(Theta(3))]; %[3x1] vector

% Trajectory interpolation (DO NOT CHANGE)
Theta_ref = zeros(3,1);
Theta_dot_ref = zeros(3,1);
for i = 1:3
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);
    Theta_dot_ref(i) = interp1(trajectory(1,:),trajectory(i+4,:),t);
end

% Gravity Compensation Control
% ???

K_p = [100 100 100]; % Proportional gain matrix containing gains K_p1 to K_p3
K_v = [25 25 25]; % Derivative gain matrix containing gains K_v1 to K_v3

%Error Calculations to determine Tau
errTheta = Theta - Theta_ref;
errTheta_dot = Theta_dot - Theta_dot_ref;
tau = [errTheta(1) errTheta_dot(1); errTheta(2) errTheta_dot(2); errTheta(3) errTheta_dot(3)]*[Kp; Kv];% control input (torques)

end

