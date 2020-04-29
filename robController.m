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
b = rob_struct.parameters.b;
m1 = rob_struct.parameters.m1;
m2 = rob_struct.parameters.m2;
m3 = rob_struct.parameters.m3;
m4 = rob_struct.parameters.m4;
l1 = rob_struct.parameters.l1;
l2 = rob_struct.parameters.l2;
l3 = rob_struct.parameters.l3;
I1 = rob_struct.parameters.I1;
I2 = rob_struct.parameters.I2;
J1 = rob_struct.parameters.J1;
J2 = rob_struct.parameters.J2;
J3 = rob_struct.parameters.J3;
    
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

K_p = []; % Proportional gain matrix containing gains K_p1 to K_p3
K_v = []; % Derivative gain matrix containing gains K_v1 to K_v3

tau = []; % control input (torque)

end

