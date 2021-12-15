function [cmdV,cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
% FEEDBACKLIN: transform Vx and Vy with respect to the inertial frame to 
% forward velocity and angular velocity
%
%   [cmdV,cmdW] = feedbackLin(cmdVx,cmdVy,theta,epsilon)
%   return the foward velocity and angular velocity command to the robot
%   while not saturate the motors (+-0.5m/s)  
% 
%   INPUTS
%       cmdVx      x-axis components of desired velocity (m/s)
%       cmdVy      y-axis components of desired velocity (m/s)
%       theta      robot frame angle (rad)
%       epsilon    distance from the center of the robot(m)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
%  
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   Zhu, Yihan

% rotational matrix from iniertial frame to robot
R_RI = [cos(theta)  sin(theta);
        -sin(theta) cos(theta)];
% [1 0; 0 1/epsilon] matrix
A = [1 0; 0 1/epsilon];
% find cmdV & cmdW
VW = A * R_RI * [cmdVx cmdVy]';
cmdV = VW(1);
cmdW = VW(2);
end 