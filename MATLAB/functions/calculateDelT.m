function delT = calculateDelT(a, b, w, qi)
% calculateDelT - Calculates the partial derivative of the coordinate
% transform matrix T, at a given pose
% 
% delT = calculateDelT(a, b, w, qi)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           qi          = 4x1 vector of the dependant cooridinates for the
%                         robot at the current pose.
% OUTPUTS:  delT        = 4x2 cell array of 1x4 partial derivatives of the
%                         transformation matrix
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

theta1 = qi(1);
beta1 = qi(2);
theta2 = qi(3);
beta2 = qi(4);
    
delT = {[(cos(beta1)*cos(beta1 - theta1))/(a*sin(beta1 - theta1)^2), (2*cos(theta1))/(a*(cos(2*beta1 - 2*theta1) - 1)), 0, 0], [(sin(beta1)*cos(beta1 - theta1))/(a*sin(beta1 - theta1)^2), (2*sin(theta1))/(a*(cos(2*beta1 - 2*theta1) - 1)), 0, 0];
        [(2*cos(beta1))/(b*(cos(2*beta1 - 2*theta1) - 1)), (cos(theta1)*cos(beta1 - theta1))/(b*sin(beta1 - theta1)^2), 0, 0], [(2*sin(beta1))/(b*(cos(2*beta1 - 2*theta1) - 1)), (sin(theta1)*cos(beta1 - theta1))/(b*sin(beta1 - theta1)^2), 0, 0];
        [0, 0, (cos(beta2)*cos(beta2 - theta2))/(a*sin(beta2 - theta2)^2), (2*cos(theta2))/(a*(cos(2*beta2 - 2*theta2) - 1))], [0, 0, (sin(beta2)*cos(beta2 - theta2))/(a*sin(beta2 - theta2)^2), (2*sin(theta2))/(a*(cos(2*beta2 - 2*theta2) - 1))];
        [0, 0, (2*cos(beta2))/(b*(cos(2*beta2 - 2*theta2) - 1)), (cos(theta2)*cos(beta2 - theta2))/(b*sin(beta2 - theta2)^2)], [0, 0, (2*sin(beta2))/(b*(cos(2*beta2 - 2*theta2) - 1)), (sin(theta2)*cos(beta2 - theta2))/(b*sin(beta2 - theta2)^2)]};
