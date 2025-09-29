function plotWSSlices (xP, yP, rectangleCoordIDs, WMdata, options)
% plotWSSlices - Plots the slices of the given workspace in the colors
% corresponding to the working modes they are from.
%  
% plotWSSlices (xP, yP, rectangleCoordIDs, WMdata, options)
% INPUTS:   xP                  = 2D array containing the X coordinates of 
%                                 all poses in the discretized workspace
%           yP                  = 2D array containing the Y coordinates of 
%                                 all poses in the discretized workspace
%           rectangleCoordIDs   = struct containing the IDs of the top row, 
%                                 bottom row, left column and right column 
%                                 of the workspace rectangle bounds
%           WMdata              = struct containing the WS direction and a
%                                 nWMs+1 x width matrix containing the 
%                                 possible working modes for each slice 
%                                 that result in the specified workspace 
%                                 rectangle, specified as ones on the line
%                                 of the corresponding line.
%           options             = name-value pairs cointaining plotting preferences
%           - rotatedWS             = look for the largest vertical WS
%                                     rectangle instead of the largest
%                                     horizontal one
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    xP double
    yP double
    rectangleCoordIDs struct
    WMdata cell
    options.rotatedWS logical = false
end

WMs = WMdata{1};
options.rotatedWS = WMdata{2};

nWMs = size(WMs,1) - 1;

WMcolors = {[1 0 0], [0 0.4470 0.7410], [0.4660 0.6740 0.1880], [0.9290 0.6940 0.1250]};

yTopID = rectangleCoordIDs.yTopID;
yBotID = rectangleCoordIDs.yBotID;
xLeftID = rectangleCoordIDs.xLeftID;
xRightID = rectangleCoordIDs.xRightID;

if options.rotatedWS
    sliceWidth = yP(1,1) - yP(2,1);
    sliceHeight = xP(yTopID, xRightID) - xP(yTopID, xLeftID);
    
    for y = yTopID : yBotID
        if y == yBotID
            thisCornerY = yP(y, xLeftID);
        else
            thisCornerY = yP(y, xLeftID) - sliceWidth/2;
        end
    
        thisCornerX = xP(y, xLeftID);
    
        if y == yTopID || y == yBotID
            thisWidth = sliceWidth/2;
        else
            thisWidth = sliceWidth;
        end
    
        thisHeight = sliceHeight/WMs(1, y-yTopID+1);
    
        for i = 2:nWMs+1
            if WMs(i, y-yTopID+1)
                rectangle('Position', [thisCornerX thisCornerY thisHeight thisWidth], FaceColor=WMcolors{i-1}, FaceAlpha=.8, EdgeColor="none");
                thisCornerX = thisCornerX + thisHeight;
            end
        end
    end
else
    sliceWidth = xP(1,2) - xP(1,1);
    sliceHeight = yP(yTopID, xLeftID) - yP(yBotID, xLeftID);
    
    for x = xLeftID : xRightID
        if x == xLeftID
            thisCornerX = xP(yBotID, x);
        else
            thisCornerX = xP(yBotID, x) - sliceWidth/2;
        end
    
        thisCornerY = yP(yBotID, x);
    
        if x == xLeftID || x == xRightID
            thisWidth = sliceWidth/2;
        else
            thisWidth = sliceWidth;
        end
    
        thisHeight = sliceHeight/WMs(1,x-xLeftID+1);
    
        for i = 2:nWMs+1
            if WMs(i, x-xLeftID+1)
                rectangle('Position', [thisCornerX thisCornerY thisWidth thisHeight], FaceColor=WMcolors{i-1}, FaceAlpha=.8, EdgeColor="none");
                thisCornerY = thisCornerY + thisHeight;
            end
        end
    end
end
