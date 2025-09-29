function plotRobot(xP, yP, xR1, yR1, xR2, yR2, w)
% plotRobot - Plots the given robot pose on the current axes
% 
% plotRobot(xP, yP, xR1, yR1, xR2, yR2, w)
% INPUTS:   xP          = 2D array containing the x-axis coordinates of the
%                         end effector
%           yP          = 2D array containing the y-axis coordinates of the
%                         end effector
%           xR1         = 2D array containing the x-axis coordinates of the
%                         elbow joint of arm 1
%           yP1         = 2D array containing the y-axis coordinates of the
%                         elbow joint of arm 1
%           xR2         = 2D array containing the x-axis coordinates of the
%                         elbow joint of arm 2
%           yP2         = 2D array containing the y-axis coordinates of the
%                         elbow joint of arm 2
%           w           = half of the base link length (nondimensional)
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

hold on
plot([-w w; xR1 xR2], [0 0; yR1 yR2], '.-', 'MarkerSize', 10, 'LineWidth', 2, 'color', 'k')
plot([xR1 xR2; xP xP], [yR1 yR2; yP yP], '.-', 'MarkerSize', 10, 'LineWidth', 2, 'color', 'k')
plot(xP, yP, '.', 'MarkerSize', 20, 'LineWidth', 2, 'color', 'r')
hold off