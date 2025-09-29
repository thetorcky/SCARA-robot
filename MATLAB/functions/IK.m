function [theta1, theta2, beta1, beta2] = IK(xP, yP, a, b, w, wm)
% IK - Solves the inverse kinematics problem for a given end effector
% position, assuming a given working mode.
%  
% [theta1, theta2, beta1, beta2] = IK(xP, yP, a, b, w, wm)
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
%           beta1       = angle of the left distal link
%           beta2       = angle of the right distal link
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

l = sqrt((xP+w).^2 + yP.^2);
l1 = (a^2 - b^2 + l.^2) ./ (2*l);
h1 = sqrt(a^2 - l1.^2);

A = l1./l;
B = wm(1)*h1./l;

rM1R1x = A.*(xP+w) + B.*yP;
rM1R1y = A.*yP + B.*(-w-xP);

rM1R1x(rM1R1x ~= real(rM1R1x)) = nan;
rM1R1y(rM1R1y ~= real(rM1R1y)) = nan;

theta1 = atan2(rM1R1y, rM1R1x);

l = sqrt((xP-w).^2 + yP.^2);
l1 = (a^2 - b^2 + l.^2) ./ (2*l);
h2 = sqrt(a^2 - l1.^2);

A = l1./l;
B = wm(2)*h2./l;

rM2R2x = A.*(xP-w) + B.*yP;
rM2R2y = A.*yP + B.*(w-xP);

rM2R2x(rM2R2x ~= real(rM2R2x)) = nan;
rM2R2y(rM2R2y ~= real(rM2R2y)) = nan;

theta2 = atan2(rM2R2y, rM2R2x);

theta1(isnan(theta2)) = nan;
theta2(isnan(theta1)) = nan;

rR1Px = xP + w - rM1R1x;
rR1Py = yP - rM1R1y;

beta1 = atan2(rR1Py, rR1Px);

rR2Px = xP - w - rM2R2x;
rR2Py = yP - rM2R2y;

beta2 = atan2(rR2Py, rR2Px);
