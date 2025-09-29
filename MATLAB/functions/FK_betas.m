function [xP, yP, xR1, yR1, xR2, yR2] = FK_betas(beta1, beta2, a, b, w, am)
% FK_betas - Calculate the position of the end efector and elbow joints based on 
% a set (or a matrix) of link angles and for a given assembly mode
% 
% [xP, yP, xR1, yR1, xR2, yR2] = FK_betas(beta1, beta2, a, b, w, am)
% INPUTS:   beta1       = 2D array containing the joint 1 input angle at 
%                         each pose 
%           beta2       = 2D array containing the joint 2 input angle at 
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

A = 2*w - 2*b*cos(beta1);
B = 2*b*sin(beta1);
C = b^2 + w^2 - 2*b*w*cos(beta1) - a^2;

D = -2*w - 2*b*cos(beta2);
E = 2*b*sin(beta2);
F = b^2 + w^2 + 2*b*w*cos(beta2) - a^2;

tau = A - D;
eps = B - E;
del = C - F;

aa = eps.^2 ./ tau.^2 + 1;
bb = (-2 * eps .* del) ./ (tau.^2) + eps ./ tau .* D - E;
cc = (del .^2) ./ (tau .^2) - del ./ tau .* D + F;

yP = ( -bb + am*sqrt(bb.^2 - 4*aa.*cc) ) ./ (2*aa);
xP = ( yP .* eps - del ) ./ tau;

xR1 = xP - b*cos(beta1);
yR1 = yP - b*sin(beta1);
xR2 = xP - b*cos(beta2);
yR2 = yP - b*sin(beta2);