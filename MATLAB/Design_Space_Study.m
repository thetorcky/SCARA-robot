% Design_Space_Study - Calculates and plots the performance atlases for all
% 5-bar robot designs in the non-dimensional design space
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

clearvars; clc; close all;
addpath(genpath('functions\'))

%%
onlyAboveX = false;
optimizeGCI = false;
rotatedWS = false;
senseAtElbows = false;
senseAtEither = true;
optimizeTrayPosition = true;
driveAtElbows = 0;
phiLimit = 90 + 90;

desiredWorkspaceWidth = 0.400;
desiredAspectTan = 256/400;

LCIlimit = 0.25;

meshSize = 0.025;
nStepsX = 101;

% meshSize = 0.10;
% nStepsX = 51;

setpoint.vMax = 3;
setpoint.aMax = 80;
setpoint.jMax = 4000;
setpoint.dwellTime = 0.1;

WMs = {[1, 1], [-1, 1], [1, -1], [-1, -1]};
am = 1;

if rotatedWS
    desiredAspectTan = 1 / desiredAspectTan;
    nStepsX = round(nStepsX * desiredAspectTan);
end

[data, options] = calculateDesignSpace(meshSize, nStepsX, WMs, am, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, ...
    onlyAboveX = onlyAboveX, optimizeGCI = optimizeGCI, rotatedWS = rotatedWS, senseAtElbows = senseAtElbows, senseAtEither = senseAtEither, optimizeTrayPosition = optimizeTrayPosition, driveAtElbows = driveAtElbows, phiLimit = phiLimit, ...
    GCI = true, ...
    GMPE = true, ...
    rectangleWidth = true, ...
    nSwitches = true, ...
    Gtau = true, ...
    Tmax = true, ...
    Trmsmax = true, ...
    elbowRange = true, ...
    trayPosition = true);

warning('on','all')

%%
close all

mask = data.rectangleWidth < 5;

indices = fieldnames(options);
for i = 1:length(indices)
    switch indices{i}
        case "Gtau"
            % plotAtlas(data.a, data.b, data.w, data.GtauAcc1, "GtauAcc1", rectangleWidthMask=mask);
            % plotAtlas(data.a, data.b, data.w, data.GtauVel1, "GtauVel1", rectangleWidthMask=mask);
            % plotAtlas(data.a, data.b, data.w, data.GtauAcc2, "GtauAcc2", rectangleWidthMask=mask);
            % plotAtlas(data.a, data.b, data.w, data.GtauVel2, "GtauVel2", rectangleWidthMask=mask);
        
        case "GMPE"
            if options.senseAtEither
                bestGMPE = min(data.GMPEsenseAtShoulders, data.GMPEsenseAtElbows);
                plotAtlas(data.a, data.b, data.w, bestGMPE, "GMPE", rectangleWidthMask=mask);
                % title("GMPE atlas - best of either");
                fprintf("Best GMPE = %f\n\n", min(bestGMPE(~mask), [], "all")*1e6);
                [a, b, w, bestGMPE] = plotSelectedRegion (data.a, data.b, data.w, bestGMPE, "<", 0.95);
                plotAtlas(data.a, data.b, data.w, bestGMPE, ["bestRegion", "GMPE"], rectangleWidthMask=mask);
            else
                plotAtlas(data.a, data.b, data.w, data.GMPE, "GMPE", rectangleWidthMask=mask);
                [a, b, w, GMPE] = plotSelectedRegion (data.a, data.b, data.w, data.GMPE, "<", 0.95);
                plotAtlas(data.a, data.b, data.w, GMPE, ["bestRegion", "GMPE"], rectangleWidthMask=mask);
            end

        case "Tmax"
            % plotAtlas(data.a, data.b, data.w, data.Tmax1, "Tmax1", rectangleWidthMask=mask);
            % title("|Tmax1| torque atlas (best tray)")
            if options.optimizeTrayPosition
                plotAtlas(data.a, data.b, data.w, data.Tmax1a, "Tmax1", rectangleWidthMask=mask);
                % title("|Tmax1a| torque atlas")
                % plotAtlas(data.a, data.b, data.w, data.Tmax1b, "Tmax1", rectangleWidthMask=mask);
                % title("|Tmax1b| torque atlas")
                fprintf("Best Tmax1a = %f\n\n", min(data.Tmax1a(~mask), [], "all"));
                [a, b, w, Tmax1a] = plotSelectedRegion (data.a, data.b, data.w, data.Tmax1a, "<", 0.99);
                plotAtlas(data.a, data.b, data.w, Tmax1a, ["bestRegion", "Tmax1a"], rectangleWidthMask=mask);
            else
                [a, b, w, Tmax1] = plotSelectedRegion (data.a, data.b, data.w, data.Tmax1, "<", 0.99);
                plotAtlas(data.a, data.b, data.w, Tmax1, ["bestRegion", "Tmax1"], rectangleWidthMask=mask);
            end
            % plotAtlas(data.a, data.b, data.w, data.Tmax1, "Tmax2", rectangleWidthMask=mask);

        case "Trmsmax"
            % plotAtlas(data.a, data.b, data.w, data.Trmsmax1, "Trmsmax1", rectangleWidthMask=mask);
            % plotAtlas(data.a, data.b, data.w, data.Trmsmax1, "Trmsmax2", rectangleWidthMask=mask);

        case "elbowRange"
            % plotAtlas(data.a, data.b, data.w, data.elbowRange1, "elbowRange1", rectangleWidthMask=mask);
            % plotAtlas(data.a, data.b, data.w, data.elbowRange1, "elbowRange2", rectangleWidthMask=mask);

        otherwise
            if isfield(data, indices{i})
                plotAtlas(data.a, data.b, data.w, data.(indices{i}), indices{i}, rectangleWidthMask=mask);
            end
            
    end
end

TileFigures