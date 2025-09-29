function JJ = calculateJacobianBetas(a, b, w, xP, yP, beta1, beta2)
% calculateJacobianBetas - Calculates the Jacobian for each pose of a given
% robot. The 2x2 Jacobian matrices are returned as pages of a 4D array
% that matches the size of the workspace defined by xP & yP.
%
% dot(theta) = JJ * dot(p)
% 
% JJ = calculateJacobianBetas(a, b, w, xP, yP, beta1, beta2)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           xP          = 2D array containing the X coordinates of all
%                         poses in the discretized workspace
%           yP          = 2D array containing the Y coordinates of all
%                         poses in the discretized workspace
%           beta1       = 2D array containing the joint 1 input angle at 
%                         each pose 
%           beta2       = 2D array containing the joint 2 input angle at 
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

JJtheta(1, 1, :, :) = (yP .* cos(beta1) - (xP + w) .* sin(beta1)) * b;
JJtheta(1, 2, :, :) = zeros(size(xP,1), size(xP,2));
JJtheta(2, 1, :, :) = JJtheta(1, 2, :, :);
JJtheta(2, 2, :, :) = (yP .* cos(beta2) + (w - xP) .* sin(beta2)) * b;

JJP(1, 1, :, :) = xP + w - b .* cos(beta1);
JJP(1, 2, :, :) = yP - b .* sin(beta1);
JJP(2, 1, :, :) = xP - w - b .* cos(beta2);
JJP(2, 2, :, :) = yP - b .* sin(beta2);

JJthetaInv = pageinv(JJtheta);

JJ = pagemtimes(JJthetaInv, JJP);