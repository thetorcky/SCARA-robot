function plotAtlas(a, b, w, atlasData, name, options)
% plotAtlas - Plots the trapezoidal atlas for a given matrix of performance 
% index data points.
%  
% plotAtlas(a, b, w, atlasData, name, options)
% INPUTS:   a           = driven link length mesh points
%           b           = floating link length mesh points
%           w           = half of the base link length mesh points
%           atlasData   = 2D matrix containing a global performance index
%                         data for all designs in the design space (= for
%                         each a-b pair)
%           name        = name of the index to be plotted. Must match with
%                         those defined below. The index name also sets the
%                         relevant options for the plotted atlas.
%           options     = name-value pairs cointaining plotting preferences
%           - rectangleWidthMask    = logical mask to omit data points
%                                     based on their rectangleWidth
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    a double
    b double
    w double
    atlasData double
    name string
    options.rectangleWidthMask 
end

if isfield(options, "rectangleWidthMask") && name(1) ~= "rectangleWidth"
    atlasData(options.rectangleWidthMask) = nan;
end

plotTrapezoid();
hold on
c = colorbar;

colorBarFlip = false;
logScale = false;
removeOutliers = false;

switch name(1)
    case "GCI"
        colorBarLabel = "GCI [-]";
        atlasTitle = "GCI atlas";

    case "GMPE"
        colorBarLabel = "GMPE [μm]";
        atlasTitle = "GMPE atlas";
        atlasData = atlasData*1e6;
        colorBarFlip = true;
        logScale = true;
        removeOutliers = true;

    case "rectangleWidth"
        colorBarLabel = "rectangleWidth [xSteps]";
        atlasTitle = "Rectangle width atlas";

    case "nSwitches"
        colorBarLabel = "nSwitches [-]";
        atlasTitle = "Number of WM switches atlas";
        colorBarFlip = true;
        % atlasData = nanMatrixOutliers(atlasData, nMAD=10);

    case "GtauAcc1"
        colorBarLabel = "GtauAcc1 [Nm/(m/s^2) = kgm]";
        atlasTitle = "GtauAcc1 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "GtauVel1"
        colorBarLabel = "GtauVel1 [Nm/(m^2/s^2) = kg]";
        atlasTitle = "GtauVel1 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "GtauAcc2"
        colorBarLabel = "GtauAcc2 [Nm/(m/s^2) = kgm]";
        atlasTitle = "GtauAcc2 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "GtauVel2"
        colorBarLabel = "GtauVel2 [Nm/(m^2/s^2) = kg]";
        atlasTitle = "GtauVel2 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "Tmax1"
        colorBarLabel = "|Tmax1| [Nm]";
        atlasTitle = "|Tmax1| torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "Tmax2"
        colorBarLabel = "|Tmax2| [Nm]";
        atlasTitle = "|Tmax2| torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "Trmsmax1"
        colorBarLabel = "Trmsmax1 [Nm]";
        atlasTitle = "Trmsmax1 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "Trmsmax2"
        colorBarLabel = "Trmsmax2 [Nm]";
        atlasTitle = "Trmsmax2 torque atlas";
        colorBarFlip = true;
        removeOutliers = true;
        logScale = true;

    case "elbowRange1"
        colorBarLabel = "elbowRange1 [deg]";
        atlasTitle = "Elbow 1 angle range atlas";
        colorBarFlip = true;
        atlasData = rad2deg(atlasData);

    case "elbowRange2"
        colorBarLabel = "elbowRange2 [deg]";
        atlasTitle = "Elbow 2 angle range atlas";
        colorBarFlip = true;
        atlasData = rad2deg(atlasData);

    case "bestRegion"
        colorBarLabel = "Better than percentage [-]";
        atlasTitle = "Best performance region for " + name(2);

    case "trayPosition"
        colorBarLabel = "Tray position [-]";
        atlasTitle = "Tray position atlas";

    otherwise
        warning("plotDesignSpace: Unknown performance index");
        return
        
end

c.Label.String = colorBarLabel;
title(atlasTitle)

if colorBarFlip
    colormap(flipud(colormap))
end

if logScale
    set(gca,'ColorScale','log')
end

if removeOutliers
    atlasData = nanMatrixOutliers(atlasData, nMAD=50);
end

axSurf = surf(a, b, w, atlasData, 'EdgeColor', 'none');

hold off
set(axSurf, 'Clipping', 'off');
fontsize(14,"points");
axis equal;
% uistack(axTrp, 'top');
aH = ancestor(axSurf,'axes');
set(aH,'XLim',[0 3]);
set(aH,'YLim',[0 3]);
set(aH,'ZLim',[0 1.5]);
ax = gca;

ax.Children = ax.Children([17:25 2:11 1 12:16]);
