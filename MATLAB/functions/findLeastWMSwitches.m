function [nSwitches, rectangleWMs] = findLeastWMSwitches(rectangleWMs)
% findLeastWMSwitches - Finds the largest rectangle in the reachable
% workspace of a robot with the performance described by the matrices in
% mats. Mats may contain multiple usable working modes as pages. A robot
% pose is considered reachable if its value in any of the pages of mats is
% not NaN. The aspect ration of the desired rectangle is inferred from the
% ratio of the discretization step size. The resulting "rectangle" has an
% equal number of rows and columns in the discretized space.
%  
% [nSwitches, rectangleWMs] = findLeastWMSwitches(rectangleWMs)
% INPUTS:  rectangleWMs = 5 x width matrix containing the IDs of all 
%                         possible working modes for each slice that result
%                         in the returned workspace rectangle. The first
%                         element in each slice represents the number of
%                         possible WMs for that slice. Each following row
%                         contains a 1 if that slice is reachable in the WM
%                         of id row-1.
% OUTPUTS: nSwitches    = the minimum number of WM switches that are
%                         required to span the width of the rectangle
%          rectangleWMs = 5 x width matrix containing the IDs of all 
%                         possible working modes for each slice that result
%                         in the returned workspace rectangle. The first
%                         row for all slices is ones, followed by a one in
%                         the row corresponding to the used id +1. 
%           
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

nSwitches = nan;

nWMs = size(rectangleWMs, 1) - 1;
width = size(rectangleWMs, 2);
dp = nan(nWMs, width);
prevWM = dp;
lastdp = zeros(nWMs, 1);

for x = 1:width
    for wm = 1:nWMs
        if(rectangleWMs(wm+1, x))
            [dp(wm,x), id] = min([lastdp(wm); lastdp+1]);
            if id == 1
                prevWM(wm,x) = wm;
            else
                prevWM(wm,x) = id-1;
            end
        end
    end
    lastdp = dp(:, x);
end

[nSwitches, id] = min(lastdp);
if isempty(nSwitches)
    nSwitches = nan;
    rectangleWMs = nan;
    return
end

rectangleWMs = zeros(nWMs+1, width);
rectangleWMs(1, :) = 1;
for x = width:-1:2
    rectangleWMs(id+1, x) = 1;
    id = prevWM(id, x);
end
rectangleWMs(id+1, 1) = 1;
