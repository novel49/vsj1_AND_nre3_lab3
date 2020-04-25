function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();
M1 = robot.m_1; % [kg]
M2 = robot.m_2; % [kg]
Mr1 = robot.m_r1; % [kg]
Mr2 = robot.m_r2; % [kg]
L1 = robot.l_1; % [m]
L2 = robot.l_2; % [m]
g = robot.g; % [m/s^2]
Lc1 = robot.l_c1; % [m]
Lc2 = robot.l_c2; % [m]
I1 = robot.i_1; % [kg*m^2]
I2 = robot.i_2; % [kg*m^2]

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [pi/3; 0; pi/2; 0];

% Control Gains (Scalar)
K_p = 0; %CHANGE THESE TO EXPERIMENT WITH GAINS
K_v = 0; %CHANGE THESE TO EXPERIMENT WITH GAINS

% Numerical Integration
t = 0:dt:t_f;

X = X_0; % initialize variable to hold state vector
X_dot = zeros(4,1); % initialize variable to hold state vector derivatives

pos = zeros(2,length(t)); %position
pos(:,1) = X_0([1 3]); %initialize
vel = zeros(2,length(t)); %velocity
acc = zeros(2,length(t)); %acceleration

for i = 1:length(t)    
    % Control torques
    tau = [0; 0];
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M1*Lc1^2 + M2*(L1 + Lc2*cos(X(3)))^2 + I1 + I2, 0; 0, M2*Lc2^2 + I2];
    V = [-2*M2*Lc2*sin(X(3))*(L1+Lc2*cos(X(3)))*X(4), 0; M2*Lc2*sin(X(3))*(L1+Lc2*cos(X(3)))*X(2), 0];
    G = [0; M2*g*Lc2*cos(X(3))];
	
    % Trapezoidal Integration
	if i == 1
		acc(:,i) = M\(tau - G); %joint accelerations
	else
		acc(:,i) = M\(tau - V*vel(:,i-1) - G); %joint accelerations
		vel(:,i) = vel(:,i-1) + 0.5*(acc(:,i) + acc(:,i-1))*dt; %joint velocities
		pos(:,i) = pos(:,i-1) + 0.5*(vel(:,i) + vel(:,i-1))*dt; %joint positions
    end

    
	% Update State Vectors
	X = [pos(1,i) vel(1,i) pos(2,i) vel(2,i)];
	X_dot = [vel(1,i) acc(1,i) vel(2,i) acc(2,i)];
	
    % Plot Energy
    
end

% Graphical Simulation
% robot.handles = drawRR([],robot);
% for i = 2:length(t)
%     setRR([],robot);
%     pause(1e-6); % adjustable pause in seconds
% end

% Plot Output





end

