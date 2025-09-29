function [T1, T2, tx, xPs, yPs, percentageTa1, percentageTa2, spacialT1, spacialT2, T1rms, T2rms] = dynamicsRunSpecificationTorques (xP, yP, Metilda, Hetilda1, Hetilda2, xStart, xStop, yStart, yStop, vMax, aMax, jMax, dwell)
% dynamicsRunSpecificationTorques - Runs the index-based dynamics
% simulation for the trajectories specified by the requirements, described
% by the elements of the start and stop vectors.
% 
% [T1, T2, tx, xPs, yPs] = dynamicsRunSimulationOnIndices(xP, yP, ...
%                           Metilda, Hetilda1, Hetilda2, xStart, xStop, ...
%                           yStart, yStop, vMax, aMax, jMax)
% INPUTS:   xP          = nSteps x nSteps matrix of X coordinates
%           yP          = nSteps x nSteps matrix of Y coordinates
%           Metilda     = nSteps x nSteps cell array of the Metilda matrix
%                         for each pose
%           Hetilda1    = nSteps x nSteps cell array of the Hetilda1 matrix
%                         for each pose, the first row of Hetilda
%           Hetilda2    = nSteps x nSteps cell array of the Hetilda2 matrix
%                         for each pose, the second row of Hetilda
%           xStart      = X-coordinate of the start point [m]
%           xStop       = X-coordinate of the stop point [m]
%           yStart      = Y-coordinate of the start point [m]
%           yStop       = Y-coordinate of the stop point [m]
%           vMax        = maximum velocity limit [m/s]
%           aMax        = maximum acceleration limit[m/s2]
%           jMax        = maximum jerk limit[m/s3]
%           dwell       = additional dwell time at the end of the
%                         trajectory [s]
% OUTPUTS:  T1          = cell array of row vectors containing the torque
%                         requirement for motor 1 [Nm]
%           T2          = cell array of row vectors containing the torque
%                         requirement for motor 2 [Nm]
%           tx          = cell array of row vectors containing sampled 
%                         time instants [s]
%           xPs         = cell array of row vectors containing the indices
%                         of the discretized workspace points used at each
%                         step of the simulation
%           yPs         = cell array of row vectors containing the indices
%                         of the discretized workspace points used at each
%                         step of the simulation
%           percentageTa1 = row vector containing the percentage of 
%                           contribution of the acceleration term to the
%                           torque of motor 1 at each time step.
%           percentageTa2 = row vector containing the percentage of 
%                           contribution of the acceleration term to the
%                           torque of motor 2 at each time step.
%           spacialT1   = row vector with the maximum torque encountered on 
%                         motor 1 at all poses (along the y axis) in the WS
%           spacialT2   = row vector with the maximum torque encountered on 
%                         motor 2 at all poses (along the y axis) in the WS
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

T1 = cell(length(xStart), 1);
T2 = T1;
tx = T1;
xPs = T1;
yPs = T1;
percentageTa1 = T1;
percentageTa2 = T1;
spacialT1 = T1;
spacialT2 = T1;
T1rms = nan(length(xStart), 1);
T2rms = T1rms;

Ts = 0.0005;

for i = 1:length(xStart)
    [T1{i}, T2{i}, tx{i}, xPs{i}, yPs{i}, percentageTa1{i}, percentageTa2{i}, spacialT1{i}, spacialT2{i}] = dynamicsRunSimulationOnIndices(xP, yP, Metilda, Hetilda1, Hetilda2, xStart(i), xStop(i), yStart(i), yStop(i), vMax, aMax, jMax, Ts);
    T1rms(i) = rms([T1{i}, zeros(1, round(dwell/Ts))]);
    T2rms(i) = rms([T2{i}, zeros(1, round(dwell/Ts))]);
end
