function [xP, yP, xR1, yR1, xR2, yR2] = FK_senseAtElbows(theta1, theta2, a, b, w, am)
% FK_senseAtElbows - Calculate the position of the end efector and elbow joints based on 
% a set (or a matrix) of joint angles and for a given assembly mode
% 
% [xP, yP, xR1, yR1, xR2, yR2] = FK_senseAtElbows(theta1, theta2, a, b, w, am)
% INPUTS:   theta1      = 2D array containing the joint 1 input angle at 
%                         each pose 
%           theta2      = 2D array containing the joint 2 input angle at 
%                         each pose 
%           a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           am          = assembly mode branch index corresponding to the
%                         selected AM: +1 or -1
% OUTPUTS:  xP          = 2D array containing the x-axis coordinates of the
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
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

l1sqared = a^2 + b^2 - 2*a*b*cos(theta1);
l2sqared = a^2 + b^2 - 2*a*b*cos(theta2);

xP = (l1sqared - l2sqared) / (4*w);
yP = am * sqrt(l1sqared - (w + xP).^2);

xP(imag(yP) ~= 0) = nan;
yP(imag(yP) ~= 0) = nan;

rho1 = atan2(yP, w + xP);
eps1 = acos((a^2 + l1sqared - b^2) ./ (2*a.*sqrt(l1sqared)));
rho2 = atan2(yP, w - xP);
eps2 = acos((a^2 + l2sqared - b^2) ./ (2*a.*sqrt(l2sqared)));

eps1(theta1 < pi) = -eps1(theta1 < pi);
eps2(theta2 > pi) = -eps2(theta2 > pi);

xR1 = a*cos(rho1 + eps1) - w;
yR1 = a*sin(rho1 + eps1);
xR2 = a*cos(pi - rho2 - eps2) + w;
yR2 = a*sin(pi - rho2 - eps2);

% norm([xR1 + w; yR1 + 0])
% norm([xR2 - w; yR2 + 0])
% 
% norm([xR1 - xP; yR1 - yP])
% norm([xR2 - xP; yR2 - yP])