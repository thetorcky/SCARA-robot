function phi = calculateElbowDeviationAngle(theta, beta)
% calculateElbowDeviationAngle - Calculate the position of the end efector and elbow joints based on 
% a set (or a matrix) of motor angles and assuming the "up" assembly mode
% 
% phi = calculateElbowDeviationAngle(theta, beta)
% INPUTS:   theta       = 2D array containing the joint input angle at 
%                         each pose 
%           beta        = 2D array containing the floating link angles at 
%                         each pose 
% OUTPUTS:  phi         = 2D array containing the elbow deviation angles at 
%                         each pose 
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

phi = pi - abs(pi - abs(theta - beta));
