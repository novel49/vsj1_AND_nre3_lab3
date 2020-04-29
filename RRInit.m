function [ robot ] = RRInit(  )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 4
% Solutions by Craig McDonald
% 
%
%    DESCRIPTION - Initialize a structure "robot" to contain important
%    robot information that will be passed into various simulation
%    functions.
%
%    ADDITIONAL CODE NEEDED:
%
%    Provide values for the missing system parameters.
%
%    Provide the transform describing the end-effector frame relative to
%    the last frame determined by D-H.
%
%    Provide the limits of the workspace.
%
% 


robot.m_1 = 1; % [kg]
robot.m_2 = 5; % [kg]
robot.m_r1 = 2.3; % [kg]
robot.m_r2 = 2.3; % [kg]
robot.l_1 = 1; % [m]
robot.l_2 = 1.41; % [m]
robot.g = 9.81; % [m/s^2]
robot.l_c1 = (robot.m_1 + 0.5*robot.m_r1)/(robot.m_1 + robot.m_r1)*robot.l_1; % [m]
robot.l_c2 = (robot.m_2 + 0.5*robot.m_r2)/(robot.m_2 + robot.m_r2)*robot.l_2; % [m]
robot.M_1 = robot.m_1 + robot.m_r1; % [kg]
robot.M_2 = robot.m_2 + robot.m_r2; % [kg]
robot.i_1 = (robot.m_1/4 + robot.m_r1/12)*robot.l_1^2; % [kg*m^2]
robot.i_2 = (robot.m_2/4 + robot.m_r2/12)*robot.l_2^2; % [kg*m^2]
robot.tool = dhtf(0, robot.l_2, 0, 0);
robot.workspace = [-2.41 2.41, -2.41 2.41, -1.41 1.41]; % only used to determine size of figure window
robot.colors = {[0,0,0.5],[0,0.5,0.5],[0,0.5,0]};

end

