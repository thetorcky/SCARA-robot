function stitchedIndex = stitchLocalIndex(pagedIndices, rectangleCoordIDs, rectangleWMs, options)
% stitchLocalIndex - Stitches the data for all poses in the stitched
% workspace following working mode switching. 
%  
% stitchedIndex = stitchLocalIndex(pagedIndices, rectangleCoordIDs, rectangleWMs)
% INPUTS:  pagedIndices = nStepsY x nStepsX x nWMs paged matrix, containing
%                         the a local performance index for nWMs different
%                         working modes.
%          rectangleCoordIDs = struct containing the IDs of the top row, 
%                         bottom row, left column and right column of the 
%                         workspace rectangle bounds.
%          rectangleWMs = 5 x width matrix containing the IDs of all 
%                         possible working modes for each slice that result
%                         in the returned workspace rectangle. The first
%                         element in each slice represents the number of
%                         possible WMs for that slice. Each following row
%                         contains a 1 if that slice is reachable in the WM
%                         of id row-1.
%          options      = name-value pairs cointaining plotting preferences
%          - rotatedWS             = look for the largest vertical WS
%                                     rectangle instead of the largest
%                                     horizontal one
% OUTPUTS: stitchedIndex = nStepsY x nStepsX matrix, containing the local
%                         performance index for the stitched together
%                         workspace
%           
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    pagedIndices double
    rectangleCoordIDs struct
    rectangleWMs double
    options.rotatedWS logical = false
end

stitchedIndex = nan(size(pagedIndices, 1), size(pagedIndices, 2));

if isnan(rectangleCoordIDs.yTopID)
    return
end

rectangleWMs = getWMids(rectangleWMs);
rectangleWMs = rectangleWMs(2, :);

if options.rotatedWS
    for y = 1:size(rectangleWMs, 2)
        stitchedIndex(rectangleCoordIDs.yTopID + y - 1, rectangleCoordIDs.xLeftID:rectangleCoordIDs.xRightID) = ...
            pagedIndices(rectangleCoordIDs.yTopID + y - 1, rectangleCoordIDs.xLeftID:rectangleCoordIDs.xRightID, rectangleWMs(y));
    end
else
    for x = 1:size(rectangleWMs, 2)
        stitchedIndex(rectangleCoordIDs.yTopID:rectangleCoordIDs.yBotID, rectangleCoordIDs.xLeftID + x - 1) = ...
            pagedIndices(rectangleCoordIDs.yTopID:rectangleCoordIDs.yBotID, rectangleCoordIDs.xLeftID + x - 1, rectangleWMs(x));
    end
end

% Smear index outside of WS
% stitchedIndex = pagedIndices(:, :, rectangleWMs(1));
% 
% for x = 1:size(rectangleWMs, 2)
%     stitchedIndex(:, rectangleCoordIDs.xLeftID + x - 1) = ...
%         pagedIndices(:, rectangleCoordIDs.xLeftID + x - 1, rectangleWMs(x));
% end
