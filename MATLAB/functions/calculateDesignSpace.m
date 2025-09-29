function [data, options] = calculateDesignSpace(meshSize, nStepsX, WMs, am, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, options)
% calculateDesignSpace - Calculates the performance characteristics of all 
% 5-bar robot designs in the non-dimensional design space, for a specified
% desired rectangular workspace size. Working mode switching is considered,
% but only in the X direction. A lower limit on the local conditioning
% index is used to determine whether a pose is reachable in given WM, for a
% particular robot. The data to be returned by the function must be
% specified by turning on the individual atlases in the options.
%  
% [data, options] = calculateDesignSpace(nStepsX, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, options)
% INPUTS:   meshSize    = the distance between two points in the a-b plane
%                         discretization
%           nStepsX     = the number of steps in X direction to be used
%                         when discretizing the 6 x 6 non-dimensional area
%           WMs         = cell array containing the branch indices for all
%                         possible working modes
%           am          = assembly mode branch index corresponding to the
%                         selected AM: +1 or -1
%           desiredWorkspaceWidth = target workspace width in meters, used
%                         to bring the non-dimensional robot design to the
%                         dimensional world for dynamics calculations
%           desiredAspectTan = tangent of the target workspace aspect
%                         ratio, used to for finding the actual 
%                         (rectangular) workspace of the robot.
%           LCIlimit    = lower limit on the local conditioning index, used
%                         to determine if a pose should be considered
%                         reachable or not. 
%           setpoint    = struct containing the setpoint parameters: the
%                         limits for velocity, acceleration, jerk and the
%                         dwell time.
%           options     = name-value pairs cointaining plotting preferences
%           - onlyAboveX            = only consider the potential WS above
%                                     the motor line. Not compatible with
%                                     rotatedWS.
%           - optimizeGCI           = instead of selecting the WM slices
%                                     that minimize the requried switches,
%                                     optimize for the highest GCI in the
%                                     stitched workspace
%           - rotatedWS             = look for the largest vertical WS
%                                     rectangle instead of the largest
%                                     horizontal one
%           - senseAtElbows         = calculate the MPE based on encoders
%                                     positioned at the elbow joints,
%                                     instead of the shoulders
%           - senseAtEither         = calculate the MPE based on encoders
%                                     positioned at the elbow joints or
%                                     at the shoulders
%           - optimizeTrayPosition  = run torque simultaions for both
%                                     tray/shuttle position options and
%                                     choose the lowest torque option
%           - driveAtElbows         = drive position selection. 1 for
%                                     driving at elbows relative to the 
%                                     ground. 2 for driving at the elbows,
%                                     between the links. 0 for driving at
%                                     the shoulders.
%           - phiLimit              = apply an angle limit (in degrees) to
%                                     the range of the elbow hinges. 180 =
%                                     no limit
%           - GCI                   = request the return of the global
%                                     conditioning index
%           - GMPE                  = request the return of the global
%                                     maximum positioning error
%           - rectangleWidth        = request the return of the
%                                     non-dimensional rectangle width
%           - nSwitches             = request the return of the number of
%                                     WM switches required to reach the
%                                     maximum actual workspace
%           - Gtau                  = request the return of the four torque
%                                     indices
%           - Tmax                  = request the return of the maximum
%                                     torques on each motor.
%           - Trmsmax               = request the return of the maximum
%                                     torques on each motor.
%           - elbowRange            = request the return of the rotational
%                                     range needed for the elbow joint on
%                                     each arm
%           - trayPosition          = request the return of the type of
%                                     tray positioning that results in the
%                                     lowest torques
% OUTPUTS:  data        = struct containing the global performance indices
%                         for all usable designs in the design space.
%           outCoordIDs = struct containing the options communicated to the
%                         function when it was called, includes the
%                         requested atlases.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    meshSize double
    nStepsX double
    WMs cell
    am double
    desiredWorkspaceWidth double
    desiredAspectTan double
    LCIlimit double
    setpoint struct
    options.onlyAboveX (1,1) logical = false
    options.optimizeGCI (1,1) logical = false
    options.rotatedWS (1,1) logical = false
    options.senseAtElbows (1,1) logical = false
    options.senseAtEither (1,1) logical = false
    options.optimizeTrayPosition (1,1) logical = false
    options.driveAtElbows (1,1) int8 = 0
    options.phiLimit (1,1) double = 180
    options.rectangleWidth (1,1) logical = false
    options.nSwitches (1,1) logical = false
    options.GCI (1,1) logical = false
    options.GMPE (1,1) logical = false
    options.Gtau (1,1) logical = false
    options.Tmax (1,1) logical = false
    options.Trmsmax (1,1) logical = false
    options.elbowRange (1,1) logical = false
    options.trayPosition (1,1) logical = false
end

[a, b] = meshgrid(0:meshSize:3);
w = 3 - a - b;
w(w>1.5) = nan;
w(w<0) = nan;

data.a = a;
data.b = b;
data.w = w;
data.meshSize = meshSize;
data.nStepsX = nStepsX;
data.LCIlimit = LCIlimit;

GCI = nan(size(a, 2), size(b, 1));
GMPE = GCI;
GMPEsenseAtShoulders = GCI;
GMPEsenseAtElbows = GCI;
rectangleWidth = GCI;
nSwitches = GCI;
elbowRange1 = GCI;
elbowRange2 = GCI;
GtauAcc1 = GCI;
GtauVel1 = GCI;
GtauAcc2 = GCI;
GtauVel2 = GCI;
Tmax1 = GCI;
Tmax2 = GCI;
Tmax1a = GCI;
Tmax2a = GCI;
Tmax1b = GCI;
Tmax2b = GCI;
Trmsmax1 = GCI;
Trmsmax2 = GCI;
trayPosition = GCI;

plotSizeX = 6;
plotSizeY = 6;

tic
fprintf("Calculation starting for %d lines.\n", length(a));

f = waitbar(0, sprintf("Calculation starting for %d lines", length(a)), 'Name', 'Calculating design space');

opts = parforOptions(gcp(), 'RangePartitionMethod', 'fixed', 'SubrangeSize', 2);

iLastfoundRectangles = 0;

for i=1:length(a)
    ppm = ParforProgressbar(length(b), 'showWorkerProgress', true);
    parfor (j=1:length(b), opts)
        if(~isnan(w(i,j)))
            d = calculateRobot(a(i,j), b(i,j), w(i,j), WMs, am, nStepsX, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, ...
                verbose = 1, onlyAboveX = options.onlyAboveX, optimizeGCI = options.optimizeGCI, rotatedWS = options.rotatedWS, senseAtElbows = options.senseAtElbows, senseAtEither = options.senseAtEither, optimizeTrayPosition = options.optimizeTrayPosition, driveAtElbows = options.driveAtElbows, phiLimit = options.phiLimit, ...
                rectangleWidth = options.rectangleWidth, ...
                nSwitches = options.nSwitches, ...
                GCI = options.GCI, ...
                GMPE = options.GMPE, ...
                Gtau = options.Gtau, ...
                Tmax = options.Tmax, ...
                Trmsmax = options.Trmsmax, ...
                elbowRange = options.elbowRange);

            if options.rectangleWidth
                rectangleWidth(i,j) = d.rectangleWidth;
            end
            if options.nSwitches
                nSwitches(i,j) = d.nSwitches;
            end
            if options.GCI
                GCI(i,j) = d.GCI;
            end
            if options.GMPE
                GMPE(i,j) = d.GMPE;
                if options.senseAtEither
                    GMPEsenseAtShoulders(i,j) = d.GMPEsenseAtShoulders;
                    GMPEsenseAtElbows(i,j) = d.GMPEsenseAtElbows;
                end
            end
            if options.Gtau
                GtauAcc1(i,j) = d.GtauAcc1;
                GtauVel1(i,j) = d.GtauVel1;
                GtauAcc2(i,j) = d.GtauAcc2;
                GtauVel2(i,j) = d.GtauVel2;
            end
            if isfield(d, 'Tmax1') && options.Tmax 
                Tmax1(i,j) = d.Tmax1;
                Tmax2(i,j) = d.Tmax2;

                Tmax1a(i,j) = d.Tmax1a;
                Tmax2a(i,j) = d.Tmax2a;
                % if options.optimizeTrayPosition
                    Tmax1b(i,j) = d.Tmax1b;
                    Tmax2b(i,j) = d.Tmax2b;
                % end
            end
            if isfield(d, 'Trmsmax1') && options.Trmsmax
                Trmsmax1(i,j) = d.Trmsmax1;
                Trmsmax2(i,j) = d.Trmsmax2;
            end
            if isfield(d, 'elbowRange1') && options.elbowRange
                elbowRange1(i,j) = d.elbowRange1;
                elbowRange2(i,j) = d.elbowRange2;
            end
            if isfield(d, 'trayPosition') && options.trayPosition
                trayPosition(i,j) = d.trayPosition;
            end
        end
        ppm.increment();
    end
    
    remainingTime = datestr(seconds(toc * (length(a)/i - 1)), 'HH:MM');
    waitmsg{1} = sprintf("Calculation done for line %d of %d", i, length(a));
    waitmsg{2} = sprintf("Remaining time: %s", remainingTime);
    waitbar(i/length(a), f, waitmsg)
    
    fprintf("Calculation done for line %d of %d.\n", i, length(a));
    toc
    fprintf("Remaining time: %s.\n", remainingTime);

    data.GCI = GCI;
    data.GMPE = GMPE;
    data.GMPEsenseAtShoulders = GMPEsenseAtShoulders;
    data.GMPEsenseAtElbows = GMPEsenseAtElbows;
    data.rectangleWidth = rectangleWidth;
    data.nSwitches = nSwitches;
    data.elbowRange1 = elbowRange1;
    data.elbowRange2 = elbowRange2;
    data.GtauAcc1 = GtauAcc1;
    data.GtauVel1 = GtauVel1;
    data.GtauAcc2 = GtauAcc2;
    data.GtauVel2 = GtauVel2;
    data.Tmax1 = Tmax1;
    data.Tmax2 = Tmax2;
    data.Tmax1a = Tmax1a;
    data.Tmax2a = Tmax2a;
    data.Tmax1b = Tmax1b;
    data.Tmax2b = Tmax2b;
    data.Trmsmax1 = Trmsmax1;
    data.Trmsmax2 = Trmsmax2;
    data.trayPosition = trayPosition;

    save("DSS_data_backup", "data", "options")                    % Backup
    data.elapsedTime = toc;
    delete(ppm);
    
    if sum(~isnan(rectangleWidth(i, :)))
        iLastfoundRectangles = i;
    end
    if i - iLastfoundRectangles >= 3
        fprintf("Found no viable rectangles in the past %d lines. Stopping.\n", i - iLastfoundRectangles);
        break
    end
end

close(f)
save("DSS_data_backup", "data", "options")                    % Backup
