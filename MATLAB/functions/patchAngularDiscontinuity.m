function [thetaStart, thetaStop] = patchAngularDiscontinuity(thetaStart, thetaStop)
% patchAngularDiscontinuity - For a given pair of angular values,
% representing the start and stop angles for a movement, this function
% updates the stop value such that shortest path is taken, even if it would
% cross the -pi/pi discontinuity.
% 
% [thetaStart, thetaStop] = patchAngularDiscontinuity(thetaStart, thetaStop)
% INPUTS:   thetaStart  = joint input angle at the start of the trajectory,
%                         defined on [-pi, pi]
%           thetaStop   = joint input angle at the end of the trajectory,
%                         defined on [-pi, pi]
% OUTPUTS:  thetaStart  = the same value as provided
%           thetaStop   = adjusted joint input angle at the end of the 
%                         trajectory, defined on [-2pi, 2pi]
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

if thetaStop - thetaStart > pi
    delta = 2*pi - abs(thetaStop - thetaStart);
    thetaStop = thetaStart - delta;
    
elseif thetaStop - thetaStart < -pi
    delta = 2*pi - abs(thetaStop - thetaStart);
    thetaStop = thetaStart + delta;

end