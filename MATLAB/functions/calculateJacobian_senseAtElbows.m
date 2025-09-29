function JJ = calculateJacobian_senseAtElbows(a, b, w, xP, yP, theta1, theta2)
% calculateJacobian - Calculates the Jacobian for each pose of a given
% robot with non-collocated encoders (encoders at the elbows). The 2x2 
% Jacobian matrices are returned as pages of a 4D array that matches the 
% size of the workspace defined by xP & yP.
%
% dot(theta) = JJ * dot(p)
%  
% JJ = calculateJacobian_senseAtElbows(a, b, w, xP, yP, theta1, theta2)
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
% OUTPUTS:  JJ          = 2 x 2 x nStepsY x nStepsX array containging the
%                         2 x 2 Jacobian matrices as pages
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

JJtheta = nan(2, 2, size(xP,1), size(xP,2));
JJP = JJtheta;

JJtheta(1, 1, :, :) = -2 * a * b * sin(theta1);
JJtheta(1, 2, :, :) = zeros(size(xP,1), size(xP,2));
JJtheta(2, 1, :, :) = JJtheta(1, 2, :, :);
JJtheta(2, 2, :, :) = -2 * a * b* sin(theta2);

JJP(1, 1, :, :) = -2*xP - 2*w;
JJP(1, 2, :, :) = -2*yP;
JJP(2, 1, :, :) = -2*xP + 2*w;
JJP(2, 2, :, :) = -2*yP;

JJthetaInv = pageinv(JJtheta);

JJ = pagemtimes(JJthetaInv, JJP);