function plotRectangle (xP, yP, rectangleCoordIDs)
% plotRectangle - Plots the bounds of the specified workspace rectangle. 
%  
% plotRectangle (xP, yP, rectangleCoordIDs)
% INPUTS:   xP                  = 2D array containing the X coordinates of 
%                                 all poses in the discretized workspace
%           yP                  = 2D array containing the Y coordinates of 
%                                 all poses in the discretized workspace
%           rectangleCoordIDs   = [yTopID, yBotID, xLeftID, xRightID] the
%                                 IDs of the top row, bottom row, left 
%                                 column and right column of the workspace 
%                                 rectangle bounds, respectively
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

hold on
plot([xP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xLeftID) xP(rectangleCoordIDs.yBotID,rectangleCoordIDs.xLeftID) xP(rectangleCoordIDs.yBotID,rectangleCoordIDs.xRightID) xP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xRightID) xP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xLeftID)], ...
     [yP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xLeftID) yP(rectangleCoordIDs.yBotID,rectangleCoordIDs.xLeftID) yP(rectangleCoordIDs.yBotID,rectangleCoordIDs.xRightID) yP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xRightID) yP(rectangleCoordIDs.yTopID,rectangleCoordIDs.xLeftID)], ...
     'm', 'LineWidth', 2)
plot(xP(round((rectangleCoordIDs.yTopID+rectangleCoordIDs.yBotID)/2), round((rectangleCoordIDs.xLeftID+rectangleCoordIDs.xRightID)/2)), ...
      yP(round((rectangleCoordIDs.yTopID+rectangleCoordIDs.yBotID)/2), round((rectangleCoordIDs.xLeftID+rectangleCoordIDs.xRightID)/2)), '.m', 'MarkerSize', 5)
hold off