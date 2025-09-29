function plotRobotAtCoords(a, b, w, wm, am, xP, yP)
% plotRobotAtCoords - Plots the given robot, in the chosen working mode, at
% a particular pose.
% 
% plotRobotAtCoords(a, b, w, wm, xP, yP)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           wm[1x2]     = vector containing the branch indices for the two
%                         arms: wm = [delta1, delta2]
%           am          = assembly mode branch index corresponding to the
%                         selected AM: +1 or -1
%           xP          = 2D array containing the X coordinates of all
%                         poses in the discretized workspace
%           yP          = 2D array containing the Y coordinates of all
%                         poses in the discretized workspace
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

hold on
[thet1, thet2] = IK(xP, yP, a, b, w, wm);
[xpp, ypp, xR1, yR1, xR2, yR2] =  FK(thet1, thet2, a, b, w, am);
plotRobot(xpp, ypp, xR1, yR1, xR2, yR2, w);
hold off
