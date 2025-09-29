function IDbounds = calculateTheoreticalIDBounds (xP, yP, a, b, w)
% calculateTheoreticalIDBounds - Calculates the IDs of the non-NaN bounds
% of the performance data of a given robot design, based on the
% intersections of the circles forming its theoretical workspace.
% 
% IDbounds = calculateTheoreticalIDBounds (xP, yP, a, b, w)
% INPUTS:   xP          = nSteps x nSteps matrix of X coordinates
%           yP          = nSteps x nSteps matrix of Y coordinates
%           a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
% OUTPUTS:  IDbounds    = struct containing the IDs of the top row, bottom 
%                         row, left column and right column of the 
%                         theoretical workspace bounds.
% 
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

if (a-b)^2-w^2+2*a*b-(a*b/w)^2 > 0
    bounds.xLeft = -a*b/w;
else
    bounds.xLeft = -(a + b - w);
end
bounds.xRight = -bounds.xLeft;

bounds.yTop = sqrt((a+b)^2 - w^2);
bounds.yBot = -bounds.yTop;

[~, IDbounds.xLeft] = min(abs(xP(1,:)-bounds.xLeft));
[~, IDbounds.xRight] = min(abs(xP(1,:)-bounds.xRight));
[~, IDbounds.yBot] = min(abs(yP(:,1)-bounds.yBot));
[~, IDbounds.yTop] = min(abs(yP(:,1)-bounds.yTop));

if IDbounds.xLeft - 1 >= 1
    IDbounds.xLeft = IDbounds.xLeft - 1;
end
if IDbounds.xRight + 1 <= size(xP, 2)
    IDbounds.xRight = IDbounds.xRight + 1;
end
if IDbounds.yTop - 1 >= 1
    IDbounds.yTop = IDbounds.yTop - 1;
end
if IDbounds.yBot + 1 <= size(xP, 1)
    IDbounds.yBot = IDbounds.yBot + 1;
end

