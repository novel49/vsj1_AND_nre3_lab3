function [ trajectory ] = createRobTrajectory( via, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

	dt = 0.1;
	t_f = 30; % final time (do not change) [s]
	t = 0:dt:t_f;
	t_goal = [5 25 30];
	pos = zeros(3,length(t));
	vel = zeros(3,length(t));
	
	for n = 1:length(t_goal)
		if n == 1
			istart = 1; %index
			[~, startjoint] = robIK(via(:,n),[0;0;0],rob); %initial joint positions
		else
			istart = find(t==t_goal(n-1));
			[~, startjoint] = robIK(via(:,n),pos(:,t==t_goal(n-1)),rob);
		end
		iend = find(t==t_goal(n)); %index
		[~, endjoint] = robIK(via(:,n+1),pos(:,t==t_goal(n)),rob); %end joint positions
		
		for j = 1:3
			pos(j,istart:iend) = linspace(startjoint(j),endjoint(j),iend-istart+1);
		end
	end
	
	vel(:,2:end) = (pos(:,2:end) - pos(:,1:end-1))/dt;
	
	trajectory(1,:) = t; %Time
	trajectory(2:4,:) = pos; %Joint angles
	trajectory(5:7,:) = vel; %Joiint velocities

end

