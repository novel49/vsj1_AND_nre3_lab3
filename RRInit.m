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


robot.m_1 = []; % [kg]
robot.m_2 = []; % [kg]
robot.m_r1 = []; % [kg]
robot.m_r2 = []; % [kg]
robot.l_1 = []; % [m]
robot.l_2 = []; % [m]
robot.g = 9.81; % [m/s^2]
robot.tool = [];
robot.workspace = []; % only used to determine size of figure window
robot.colors = {[0,0,0],[0,0,0],[0,0,0]};

end

