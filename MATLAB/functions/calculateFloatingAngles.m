function [beta1, beta2] = calculateFloatingAngles(a, b, w, xP, yP, theta1, theta2)
% calculateFloatingAngles - Calculate the angles made by the floating links
% with the horiontal. Returns an angle between 0 and pi.
% 
% [beta1, beta2] = calculateFloatingAngles(a, b, w, xP, yP, theta1, theta2)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           xP          = 2D array containing the X coordinates of all
%                         poses in the discretized workspace
%           yP          = 2D array containing the Y coordinates of all
%                         poses in the discretized workspace
%           theta1      = 2D array containing the joint 1 input angle at 
%                         each pose 
%           theta2      = 2D array containing the joint 2 input angle at 
%                         each pose 
% OUTPUTS:  beta1       = 2D array containing the angles of foloating link
%                         1 at each pose
%           beta2       = 2D array containing the angles of foloating link
%                         2 at each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

xR1 = a*cos(theta1) - w;
yR1 = a*sin(theta1);
xR2 = a*cos(theta2) + w;
yR2 = a*sin(theta2);

x1Delta = xP - xR1;
y1Delta = yP - yR1;
x2Delta = xP - xR2;
y2Delta = yP - yR2;

x1Delta(~isreal(x1Delta)) = nan;
y1Delta(~isreal(y1Delta)) = nan;
x2Delta(~isreal(x2Delta)) = nan;
y2Delta(~isreal(y2Delta)) = nan;

beta1 = atan2(y1Delta, x1Delta);
beta2 = atan2(y2Delta, x2Delta);
