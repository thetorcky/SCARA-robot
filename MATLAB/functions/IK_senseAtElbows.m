function [theta1, theta2] = IK_senseAtElbows(xP, yP, a, b, w, wm)
% IK - Solves the inverse kinematics problem for a robot with 
% non-collocated encoders (encoders at the elbows), for a given end 
% effector position, assuming a given working mode.
%  
% [theta1, theta2] = IK_senseAtElbows(xP, yP, a, b, w, wm)
% INPUTS:   xP          = x-coordinate of the end effector
%           yP          = y-coordinate of the end effector
%           a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           wm[1x2]     = vector containing the branch indices for the two
%                         arms: wm = [delta1, delta2]
% 
% OUTPUTS:  theta1      = angle of the left motor
%           theta2      = angle of the right motor
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

theta1 = acos((a^2 + b^2 - ((xP+w).^2 + yP.^2))/2/a/b);
theta2 = acos((a^2 + b^2 - ((xP-w).^2 + yP.^2))/2/a/b);

if wm(1) == -1
    theta1 = 2*pi - theta1;
end

if wm(2) == -1
    theta2 = 2*pi - theta2;
end
theta1(imag(theta1) ~= 0) = nan;
theta2(imag(theta2) ~= 0) = nan;

theta1(isnan(theta2)) = nan;
theta2(isnan(theta1)) = nan;
