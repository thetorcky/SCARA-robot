function JJext = calculateJacobianExtended(a, b, w, q)
% calculateJacobianExtended - Calculates the extended Jacobian for the 
% given poses of a robot.
%
% dot(theta, beta) = JJ * dot(p)
% 
% JJext = calculateJacobianExtended(a, b, w, xP, yP)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           q           = coordinate vector of the assembled system at a 
%                         given pose = [theta1; beta1; theta1; beta1]
% OUTPUTS:  JJext       = 4 x 2 x nStepsY x nStepsX array containging the
%                         4 x 2 Jacobian matrices as pages
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

theta1 = q(1);
beta1 = q(2);
theta2 = q(3);
beta2 = q(4);

JJext = nan(4, 2, 1, 1);

JJext(1, 1, :, :) = cos(beta1) / (a * sin(beta1 - theta1));
JJext(1, 2, :, :) = sin(beta1) / (a * sin(beta1 - theta1));
JJext(2, 1, :, :) = -cos(theta1) / (b * sin(beta1 - theta1));
JJext(2, 2, :, :) = -sin(theta1) / (b * sin(beta1 - theta1));
JJext(3, 1, :, :) = cos(beta2) / (a * sin(beta2 - theta2));
JJext(3, 2, :, :) = sin(beta2) / (a * sin(beta2 - theta2));
JJext(4, 1, :, :) = -cos(theta2) / (b * sin(beta2 - theta2));
JJext(4, 2, :, :) = -sin(theta2) / (b * sin(beta2 - theta2));
