function [  ] = simulateRR()
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

	
%%%%%%% System Parameters to play with %%%%%%%
	isControl = 1; %binary switch for control

	X_0 = [pi/3; 0; pi/2; 0]; %initial conditions
	X_goal = [0; 0; pi/2; 0]; %target state configuration

	% Underdamped Response
	K_p = 100;
	K_v = 25;

% 	% Critical Response
% 	K_p = 100;
% 	K_v = 65.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	% Initialize robot
	robot = RRInit();
	M1 = robot.M_1; % [kg]
	M2 = robot.M_2; % [kg]
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
	dt = 0.001; % [s], smaller timestep reduces energy error, but takes longer
	t_f = 10; % [s]

	% Numerical Integration
	t = 0:dt:t_f;

	X = X_0; % initialize variable to hold state vector
	X_dot = zeros(4,1); % initialize variable to hold state vector derivatives

	pos = zeros(2,length(t)); %position
	pos(:,1) = X_0([1 3]); %initial conditions
	vel = zeros(2,length(t)); %velocity
	vel(:,1) = X_0([2 4]); %initial conditions
	acc = zeros(2,length(t)); %acceleration
	PE = zeros(1,length(t)); %potential energy
	KE = zeros(1,length(t)); %kinetic energy

	for i = 1:length(t)    
		% Control torques
		err = X - X_goal;
		tau = -isControl*[err(1) err(2); err(3) err(4)]*[K_p; K_v];

		% Apply joint torque limits
		tau(tau>tau_max) = tau_max;
		tau(tau<-tau_max) = -tau_max;

		% Dynamic Model
		M = [M1*Lc1^2 + M2*(L1 + Lc2*cos(X(3)))^2 + I1 + I2, 0; 0, M2*Lc2^2 + I2];
		C = [-2; 1]*M2*Lc2*sin(X(3))*(L1+Lc2*cos(X(3)))*X(2)*X(4);
		G = [0; M2*g*Lc2*cos(X(3))];

		% Trapezoidal Integration
		if i == 1
			acc(:,i) = M\(tau - G); %joint accelerations
		else
			acc(:,i) = M\(tau - C - G); %joint accelerations
			vel(:,i) = vel(:,i-1) + 0.5*(acc(:,i) + acc(:,i-1))*dt; %joint velocities
			pos(:,i) = pos(:,i-1) + 0.5*(vel(:,i) + vel(:,i-1))*dt; %joint positions
		end


		% Update State Vectors
		X = [pos(1,i); vel(1,i); pos(2,i); vel(2,i)];
		X_dot = [vel(1,i); acc(1,i); vel(2,i); acc(2,i)];

		% Calculate Energy
		% Plotting within a loop is inefficient and unnecessary in this scenario.
		PE(i) = M2*g*sin(X(3))*Lc2;
		KE(i) = 0.5*vel(:,i)'*M*vel(:,i);
	end

	% Plot Output
	figure()
	plot(t,pos)
		xlabel("Time t, s")
		ylabel("Joint Angle \theta, rad")
		title("Joint Positions Over Time")
		legend("\theta_1", "\theta_2")
		xticks(0:2:10)
		grid on

	figure()
	plot(t,vel)
		xlabel("Time t, s")
		ylabel("Joint Velocity \theta', rad/s")
		title("Joint Velocities Over Time")
		legend("\theta_1'", "\theta_2'")
		xticks(0:2:10)
		grid on

	figure()
	plot(t,PE,t,KE,t,PE+KE,'k')
		xlabel("Time t, s")
		ylabel("Energy, Joules")
		title("System Energy Over Time")
		legend("Potential Energy", "Kinetic Energy", "Total Energy")
		xticks(0:2:10)
		grid on

	%Graphical Simulation
	robot.handles = drawRR(pos(:,1),robot);
	for i = 2:length(t)
		setRR(pos(:,i),robot);
	end

end

