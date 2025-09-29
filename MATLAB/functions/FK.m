function [xP, yP, xR1, yR1, xR2, yR2] = FK(theta1, theta2, a, b, w, am)
% FK - Calculate the position of the end efector and elbow joints based on 
% a set (or a matrix) of motor angles and for a given assembly mode
% 
% [xP, yP, xR1, yR1, xR2, yR2] = FK(theta1, theta2, a, b, w, am)
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

xR1 = a*cos(theta1) - w;
yR1 = a*sin(theta1);
xR2 = a*cos(theta2) + w;
yR2 = a*sin(theta2);

f = nan(2, 1, size(theta1,1), size(theta1,2));
h = f;
R1 = f;

f(1,1,:,:) = (xR2 - xR1)/2;
f(2,1,:,:) = (yR2 - yR1)/2;

R1(1,1,:,:) = xR1;
R1(2,1,:,:) = yR1;

fnorm(:,:) = pagenorm(f);

hnorm = sqrt(b^2 - fnorm.^2);

h1(:,:) = -f(2,1,:,:);
h2(:,:) = f(1,1,:,:);

h1 = h1 .* hnorm ./ fnorm;
h2 = h2 .* hnorm ./ fnorm;

h(1,1,:,:) = h1;
h(2,1,:,:) = h2;

PP = R1 + f + am * h;

xP(:,:) = PP(1,1,:,:);
yP(:,:) = PP(2,1,:,:);
