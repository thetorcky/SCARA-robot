function [outWidth, outCoordIDs, outWMs] = findStitchedRectangle (mats, options)
% findStitchedRectangle - Finds the largest rectangle in the reachable
% workspace of a robot with the performance described by the matrices in
% mats. Mats may contain multiple usable working modes as pages. A robot
% pose is considered reachable if its value in any of the pages of mats is
% not NaN. The aspect ration of the desired rectangle is inferred from the
% ratio of the discretization step size. The resulting "rectangle" has an
% equal number of rows and columns in the discretized space.
%  
% [outWidth, outCoordIDs, outWMs] = findStitchedRectangle (mats)
% INPUTS:   mats        = nStepsY x nStepsX x nWMs paged matrix, containing
%                         the chosen performance index for nWMs different
%                         working modes. NaN values are considered
%                         unreachable, but any other real value is OK.
%           options     = name-value pairs cointaining plotting preferences
%           - verbose           = integer between 0 and 2 signifing the
%                                 desired level of console output
%           - optimizeGCI       = optimize slice selection for the highest
%                                 GCI. Results in only one sequence of WMs.
% OUTPUTS:  outWidth    = the width of the largest rectangle in terms of
%                         its number of columns in the discretization
%           outCoordIDs = struct containing the IDs of the top row, bottom 
%                         row, left column and right column of the 
%                         workspace rectangle bounds.
%           outWMs      = 5 x outWidth matrix containing the IDs of all 
%                         possible working modes for each slice that result
%                         in the returned workspace rectangle. The first
%                         element in each slice represents the number of
%                         possible WMs for that slice. Each following row
%                         contains a 1 if that slice is reachable in the WM
%                         of id row-1.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

    arguments
        mats double
        options.verbose double = 0
        options.optimizeGCI logical = false
    end

    outWidth = 0;
    outCoordIDs.yTopID = nan;
    outCoordIDs.yBotID = nan;
    outCoordIDs.xLeftID = nan;
    outCoordIDs.xRightID = nan;
    outWMs = 0;

    for yBotID = size(mats,1):-1:1
        if yBotID < outWidth
            break
        end
        if options.verbose >= 2
            fprintf("Searching for yBotID = %d.\n", yBotID);
        end
        for yTopID = 1 : min(yBotID - outWidth + 1, yBotID)
            width = yBotID - yTopID + 1;

            [xLeftID, xRightID, WMs] = tryTheseYs(mats, yBotID, yTopID, optimizeGCI = options.optimizeGCI);

            if ~isnan(xLeftID) && width >= outWidth
                if options.verbose >= 1
                    fprintf("-- Found rectangle with width = %d.\n", width);
                end
                outWidth = width;
                outCoordIDs.yTopID = yTopID;
                outCoordIDs.yBotID = yBotID;
                outCoordIDs.xLeftID = xLeftID;
                outCoordIDs.xRightID = xRightID;
                outWMs = WMs;
            end
        end
    end
    if outWidth == 0
        outWidth = nan;
    end
end

function [xLeftID, xRightID, dp] = tryTheseYs(mats, yBotID, yTopID, options)

    arguments
        mats double
        yBotID double
        yTopID double
        options.optimizeGCI logical = false
    end
    
    matsSlice = mats(yTopID:yBotID, :, :);
    width = yBotID - yTopID + 1;
    xLeftID = nan;
    xRightID = nan;
    dp = nan(size(mats,3)+1, width);
 
    if width > size(mats,2)
        return
    end

    xLeftID = 1;
    for xRightID = xLeftID : xLeftID + width - 2
        dp(:, xRightID+1) = findGoodSlice(matsSlice, xRightID, optimizeGCI = options.optimizeGCI);
    end

    for xLeftID = xLeftID : size(mats,2) - width + 1
        xRightID = xLeftID + width - 1;
        dp(:, 1) = [];
        dp(:, width) = findGoodSlice(matsSlice, xRightID, optimizeGCI = options.optimizeGCI);
        if ~anyEq(dp(1,:), 0)
            return 
        end
    end

    xLeftID = nan;
    xRightID = nan;
end

function WMs = findGoodSlice(matsSlice, xID, options)

    arguments
        matsSlice double
        xID double
        options.optimizeGCI logical = false
    end

    nWMs = size(matsSlice,3);
    WMs = zeros(nWMs+1, 1);
    WMs(1) = 0;
    maxLCIsum = 0;
    wm = nan;

    for i = 1:nWMs
        if(options.optimizeGCI)
            LCIsum = sum(matsSlice(:, xID, i), "all");
            if ~isnan(LCIsum) && LCIsum - maxLCIsum > 1e-5
                maxLCIsum = LCIsum;
                WMs(1) = 1;
                wm = i;
            end
        else
            if ~anyEq(matsSlice(:, xID, i), nan)
                WMs(1) = WMs(1) + 1;
                WMs(i+1) = 1;
            end
        end
    end
    
    if ~isnan(wm)
        WMs(wm + 1) = 1;
    end
end