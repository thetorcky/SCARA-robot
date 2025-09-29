function data = calculateRobot(a, b, w, WMs, am, nStepsX, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, options)
% calculateRobot - Calculates the performance characteristics of all 
% 5-bar robot designs in the non-dimensional design space, for a specified
% desired rectangular workspace size. Working mode switching is considered,
% but only in the X direction. A lower limit on the local conditioning
% index is used to determine whether a pose is reachable in given WM, for a
% particular robot. The data to be returned by the function must be
% specified by turning on the individual atlases in the options.
%  
% data = calculateRobot(a, b, w, WMs, nStepsX, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, options)
% INPUTS:   a           = driven link length of the robot
%           b           = floating link length of the robot
%           w           = half of the base link length of the robot
%           WMs         = cell array containing the branch indices for all
%                         possible working modes
%           am          = assembly mode branch index corresponding to the
%                         selected AM: +1 or -1
%           nStepsX     = the number of steps in X direction to be used
%                         when discretizing the 6 x 6 non-dimensional area
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
%           - verbose               = integer between 0 and 2 signifing the
%                                     desired level of console output
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
%           - all                   = request the return of all local and
%                                     global indices listed below
%           - rectangleWidth        = request the return of the
%                                     non-dimensional rectangle width
%           - nSwitches             = request the return of the number of
%                                     WM switches required to reach the
%                                     maximum actual workspace
%           - GCI                   = request the return of the global
%                                     conditioning index
%           - GMPE                  = request the return of the global
%                                     maximum positioning error
%           - Gtau                  = request the return of the four torque
%                                     indices
%           - Tmax                  = request the return of the maximum
%                                     torques on each motor.
%           - Trmsmax               = request the return of the maximum
%                                     torques on each motor.
%           - elbowRange            = request the return of the rotational
%                                     range needed for the elbow joint on
%                                     each arm
%           - ...plus other local indices...
% OUTPUTS:  data        = struct containing all the local and global 
%                         performance indices requested in the options, for
%                         the given robot design
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    a double
    b double
    w double
    WMs cell
    am double
    nStepsX double
    desiredWorkspaceWidth double
    desiredAspectTan double
    LCIlimit double
    setpoint struct
    options.verbose double = 2
    options.onlyAboveX (1,1) logical = false
    options.optimizeGCI (1,1) logical = false
    options.rotatedWS (1,1) logical = false
    options.senseAtElbows (1,1) logical = false
    options.senseAtEither (1,1) logical = false
    options.optimizeTrayPosition (1,1) logical = false
    options.driveAtElbows (1,1) int8 = 0
    options.phiLimit (1,1) double = 180
    options.all (1,1) logical = false
    options.rectangleWidth (1,1) logical = false
    options.nSwitches (1,1) logical = false
    options.thetas (1,1) logical = false
    options.betas (1,1) logical = false
    options.phis (1,1) logical = false
    options.elbowRange (1,1) logical = false
    options.LCI (1,1) logical = false
    options.GCI (1,1) logical = false
    options.MPE (1,1) logical = false
    options.GMPE (1,1) logical = false
    options.taus (1,1) logical = false
    options.Gtau (1,1) logical = false
    options.Tsims (1,1) logical = false
    options.Tspecs (1,1) logical = false
    options.Tmax (1,1) logical = false
    options.Trmsmax (1,1) logical = false
end

warning('off','MATLAB:illConditionedMatrix')
warning('off','MATLAB:singularMatrix')
warning('off','MATLAB:nearlySingularMatrix')

if abs(w - (3 - a - b)) > 1e-5
    warning("Normalized link lengths appear incompatible (w ~= 3 - a - b)")
end

plotSizeX = 6;
plotSizeY = 6;

stepSizeX = plotSizeX / nStepsX;
stepSizeY = desiredAspectTan*stepSizeX;
nStepsY = fix(plotSizeY/stepSizeY);

for i = 1:length(WMs)
    % Retrieve performance for this WM
    r(i) = calculateAllPoses(a, b, w, nStepsX, nStepsY, stepSizeX, stepSizeY, WMs{i}, am, senseAtElbows = options.senseAtElbows, senseAtEither = options.senseAtEither, driveAtElbows = options.driveAtElbows);
    % Build 3-D data matrices with all WMs
    theta1(: ,: ,i) = r(i).theta1;
    theta2(: ,: ,i) = r(i).theta2;
    beta1(: ,: ,i) = r(i).beta1;
    beta2(: ,: ,i) = r(i).beta2;
    phi1(: ,: ,i) = calculateElbowDeviationAngle(theta1(: ,: ,i), beta1(: ,: ,i));
    phi2(: ,: ,i) = calculateElbowDeviationAngle(theta2(: ,: ,i), beta2(: ,: ,i));
    LCI(: ,: ,i) = r(i).LCI;
    MPE(: ,: ,i) = r(i).MPE;
    if options.senseAtEither
        MPEsenseAtShoulders(: ,: ,i) = r(i).MPEsenseAtShoulders;
        MPEsenseAtElbows(: ,: ,i) = r(i).MPEsenseAtElbows;
    end
    % LPI(: ,: ,i) = r(i).LPI;
    % LTT(: ,: ,i) = r(i).LTT;
    % LSI(: ,: ,i) = r(i).LSI;
    % LMI(: ,: ,i) = r(i).LMI;
end

xP = r(1).xP;
yP = r(1).yP;

data.xP = xP;
data.yP = yP;

LCInan = LCI;
LCInan(LCInan < LCIlimit) = nan;

LCInan(phi1 > deg2rad(options.phiLimit) | phi2 > deg2rad(options.phiLimit)) = nan;

%% Find rectangle
IDbounds = calculateTheoreticalIDBounds(xP, yP, a, b, w);

if options.onlyAboveX
    if options.rotatedWS
        error("Conflicting parameters onlyAboveX and rotatedWS are both selected!")
    end
    [rectangleWidth, rectangleCoordIDs, allRectangleWMs] = findStitchedRectangle(LCInan(IDbounds.yTop:fix((IDbounds.yBot+IDbounds.yTop)/2), IDbounds.xLeft:IDbounds.xRight, :), verbose = options.verbose, optimizeGCI = options.optimizeGCI);

elseif options.rotatedWS
    [rectangleWidth, rectangleCoordIDs, allRectangleWMs] = findStitchedRectangle(pagetranspose(LCInan(IDbounds.yTop:IDbounds.yBot, IDbounds.xLeft:IDbounds.xRight, :)), verbose = options.verbose, optimizeGCI = options.optimizeGCI);
    rectangleCoordIDsTransposed.yTopID = rectangleCoordIDs.xLeftID;
    rectangleCoordIDsTransposed.yBotID = rectangleCoordIDs.xRightID;
    rectangleCoordIDsTransposed.xLeftID = rectangleCoordIDs.yTopID;
    rectangleCoordIDsTransposed.xRightID = rectangleCoordIDs.yBotID;
    rectangleCoordIDs = rectangleCoordIDsTransposed;
    
else
    [rectangleWidth, rectangleCoordIDs, allRectangleWMs] = findStitchedRectangle(LCInan(IDbounds.yTop:IDbounds.yBot, IDbounds.xLeft:IDbounds.xRight, :), verbose = options.verbose, optimizeGCI = options.optimizeGCI);
end

[nSwitches, rectangleWMs] = findLeastWMSwitches(allRectangleWMs);

rectangleCoordIDs.yTopID = rectangleCoordIDs.yTopID + IDbounds.yTop - 1;
rectangleCoordIDs.yBotID = rectangleCoordIDs.yBotID + IDbounds.yTop - 1;
rectangleCoordIDs.xLeftID = rectangleCoordIDs.xLeftID + IDbounds.xLeft - 1;
rectangleCoordIDs.xRightID = rectangleCoordIDs.xRightID + IDbounds.xLeft - 1;

if(rectangleWidth > 1)
    rectangleWidthNondim = (xP(1, rectangleCoordIDs.xRightID) - xP(1, rectangleCoordIDs.xLeftID));
    if options.rotatedWS 
        scalingD = desiredWorkspaceWidth * 1/desiredAspectTan / rectangleWidthNondim;
    else
        scalingD = desiredWorkspaceWidth / rectangleWidthNondim;
    end
    workspaceDistY = yP(rectangleCoordIDs.yBotID, 1) * scalingD;
else
    rectangleWidthNondim = nan;
    scalingD = nan;
    workspaceDistY = nan;
end

data.rectangleCoordIDs = rectangleCoordIDs;
data.allRectangleWMs = allRectangleWMs;
data.rectangleWMs = rectangleWMs;

if options.rectangleWidth || options.all
    data.rectangleWidth = rectangleWidth;
    data.scalingD = scalingD;
end
if options.nSwitches || options.all
    data.nSwitches = nSwitches;
end

xPscaled = xP*scalingD;
yPscaled = yP*scalingD;

data.xPscaled = xPscaled;
data.yPscaled = yPscaled;

A = a*scalingD;
B = b*scalingD;
W = w*scalingD;

if A - 1.025 < 1e-4 && B - 1.625 < 1e-4
    % robot = dynamicsAssignProperties(A, B, W, [0.22467, 0.30898, 0.19880, 0.19301, 0.200], [0.08774, 0.13777, 0.08734, 0.12641], [736812.87e-9, 2394096.65e-9, 670149.16e-9, 1671839.03e-9]); % TC1-GMPE-flexure-LS3drafts
    robot = dynamicsAssignProperties(A, B, W, [8.482, 2.348, 7.656, 0.903, 0.734], [0.032, 0.077, 0.022, 0.089], [49703322.22e-9, 28253392.69e-9, 37587682.91e-9, 11812245.71e-9]); % PB
else
    robot = dynamicsAssignProperties(A, B, W);
end
data.robot = robot;

theta1stitched = stitchLocalIndex(theta1, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
theta2stitched = stitchLocalIndex(theta2, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
beta1stitched = stitchLocalIndex(beta1, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
beta2stitched = stitchLocalIndex(beta2, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
LCIstitched = stitchLocalIndex(LCI, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
MPE = MPE * scalingD;
MPEstitched = stitchLocalIndex(MPE, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
if options.senseAtEither
    MPEsenseAtShoulders = MPEsenseAtShoulders * scalingD;
    MPEsenseAtElbows = MPEsenseAtElbows * scalingD;
    MPEsenseAtShouldersstitched = stitchLocalIndex(MPEsenseAtShoulders, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
    MPEsenseAtElbowsstitched = stitchLocalIndex(MPEsenseAtElbows, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
end

if options.thetas || options.all
    data.theta1 = theta1;
    data.theta2 = theta2;
    data.theta1stitched = theta1stitched;
    data.theta2stitched = theta2stitched;
end

if options.betas || options.all
    data.beta1 = beta1;
    data.beta2 = beta2;
    data.beta1stitched = beta1stitched;
    data.beta2stitched = beta2stitched;
end

if options.phis || options.elbowRange || options.all
    phi1stitched = stitchLocalIndex(phi1, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);
    phi2stitched = stitchLocalIndex(phi2, rectangleCoordIDs, rectangleWMs, rotatedWS = options.rotatedWS);

    if options.phis || options.all
        data.phi1stitched = phi1stitched;
        data.phi2stitched = phi2stitched;
    end
    if options.elbowRange || options.all
        data.elbowRange1 = calculateGlobalIndex(phi1stitched, "elbowRange", boundIDs = rectangleCoordIDs);
        data.elbowRange2 = calculateGlobalIndex(phi2stitched, "elbowRange", boundIDs = rectangleCoordIDs);
    end
end

if options.LCI || options.all
    data.LCI = LCI;
    data.LCInan = LCInan;
    data.LCIstitched = LCIstitched;
end
if options.GCI || options.all
    data.GCI = calculateGlobalIndex(LCIstitched, "GCI", boundIDs = rectangleCoordIDs);
end

if options.MPE || options.all
    data.MPE = MPE;
    data.MPEstitched = MPEstitched;
    if options.senseAtEither
        data.MPEsenseAtShoulders = MPEsenseAtShoulders;
        data.MPEsenseAtShouldersstitched = MPEsenseAtShouldersstitched;
        data.MPEsenseAtElbows = MPEsenseAtElbows;
        data.MPEsenseAtElbowsstitched = MPEsenseAtElbowsstitched;
    end
end
if options.GMPE || options.all
    data.GMPE = calculateGlobalIndex(MPEstitched, "GMPE", boundIDs = rectangleCoordIDs);
    if options.senseAtEither
        data.GMPEsenseAtShoulders = calculateGlobalIndex(MPEsenseAtShouldersstitched, "GMPE", boundIDs = rectangleCoordIDs);
        data.GMPEsenseAtElbows = calculateGlobalIndex(MPEsenseAtElbowsstitched, "GMPE", boundIDs = rectangleCoordIDs);
    end
end

if options.taus || options.Gtau || options.Tsims || options.Tspecs || options.Tmax || options.Trmsmax || options.all
    q = cat(3, theta1stitched, beta1stitched, theta2stitched, beta2stitched);
    [tauAcc1, tauVel1, tauAcc2, tauVel2, Metilda, Hetilda1, Hetilda2] = calculateTorqueIndices(robot, xP, yP, q, boundIDs = rectangleCoordIDs, driveAtElbows = options.driveAtElbows);

    if options.taus || options.all
        data.tauAcc1 = tauAcc1;
        data.tauVel1 = tauVel1;
        data.tauAcc2 = tauAcc2;
        data.tauVel2 = tauVel2;
    end
    if options.Gtau || options.all
        data.GtauAcc1 = calculateGlobalIndex(tauAcc1, "Gtau", boundIDs = rectangleCoordIDs);
        data.GtauVel1 = calculateGlobalIndex(tauVel1, "Gtau", boundIDs = rectangleCoordIDs);
        data.GtauAcc2 = calculateGlobalIndex(tauAcc2, "Gtau", boundIDs = rectangleCoordIDs);
        data.GtauVel2 = calculateGlobalIndex(tauVel2, "Gtau", boundIDs = rectangleCoordIDs);
    end
    if (options.Tsims || options.Tspecs || options.Tmax || options.Trmsmax || options.all) && rectangleWidth > 1
        
        if ( options.rotatedWS && (abs(xP(1, rectangleCoordIDs.xRightID)) > abs(xP(1, rectangleCoordIDs.xLeftID))) ) || ...
           ( ~options.rotatedWS && (abs(yP(rectangleCoordIDs.yBotID, 1)) > abs(yP(rectangleCoordIDs.yTopID, 1))) )
            rectangleCoordIDsFlipped.yTopID = rectangleCoordIDs.yBotID;
            rectangleCoordIDsFlipped.yBotID = rectangleCoordIDs.yTopID;
            rectangleCoordIDsFlipped.xLeftID = rectangleCoordIDs.xRightID;
            rectangleCoordIDsFlipped.xRightID = rectangleCoordIDs.xLeftID;
            rectangleCoordIDs = rectangleCoordIDsFlipped;
        end
        
        if options.rotatedWS
            yStarta = yPscaled(rectangleCoordIDs.yTopID:sign(rectangleCoordIDs.yBotID-rectangleCoordIDs.yTopID):rectangleCoordIDs.yBotID, 1);
            yStopa = yStarta;
            xStarta = xPscaled(1, rectangleCoordIDs.xRightID) * ones(1, length(yStarta));
            xStopa = xStarta - 0.180;

            yStartb = yPscaled(rectangleCoordIDs.yTopID:sign(rectangleCoordIDs.yBotID-rectangleCoordIDs.yTopID):rectangleCoordIDs.yBotID, 1);
            yStopb = yStartb;
            xStartb = xPscaled(1, rectangleCoordIDs.xLeftID) * ones(1, length(yStartb));
            xStopb = xStartb + 0.180;
        else
            xStarta = xPscaled(1, rectangleCoordIDs.xLeftID:sign(rectangleCoordIDs.xRightID-rectangleCoordIDs.xLeftID):rectangleCoordIDs.xRightID);
            xStopa = xStarta;
            yStarta = yPscaled(rectangleCoordIDs.yBotID, 1) * ones(1, length(xStarta));
            yStopa = yStarta + 0.180;

            xStartb = xPscaled(1, rectangleCoordIDs.xLeftID:sign(rectangleCoordIDs.xRightID-rectangleCoordIDs.xLeftID):rectangleCoordIDs.xRightID);
            xStopb = xStartb;
            yStartb = yPscaled(rectangleCoordIDs.yTopID, 1) * ones(1, length(xStartb));
            yStopb = yStartb - 0.180;
        end
        
        [T1a, T2a, txa, xPsa, yPsa, ~, ~, spacialT1a, spacialT2a, Trms1a, Trms2a] = dynamicsRunSpecificationTorques (xPscaled, yPscaled, Metilda, Hetilda1, Hetilda2, xStarta, xStopa, yStarta, yStopa, setpoint.vMax, setpoint.aMax, setpoint.jMax, setpoint.dwellTime);
        % if options.optimizeTrayPosition
            [T1b, T2b, txb, xPsb, yPsb, ~, ~, spacialT1b, spacialT2b, Trms1b, Trms2b] = dynamicsRunSpecificationTorques (xPscaled, yPscaled, Metilda, Hetilda1, Hetilda2, xStartb, xStopb, yStartb, yStopb, setpoint.vMax, setpoint.aMax, setpoint.jMax, setpoint.dwellTime);
        % end
        
        Tmax1a = calculateGlobalIndex(cell2mat(spacialT1a), "Tmax");
        Tmax2a = calculateGlobalIndex(cell2mat(spacialT2a), "Tmax");
        % if options.optimizeTrayPosition
            Tmax1b = calculateGlobalIndex(cell2mat(spacialT1b), "Tmax");
            Tmax2b = calculateGlobalIndex(cell2mat(spacialT2b), "Tmax");
        % end

        if ~options.optimizeTrayPosition || (max(Tmax1a, Tmax2a) <= max(Tmax1b, Tmax2b)) 
            yStart = yStarta;
            yStop = yStopa;
            xStart = xStarta;
            xStop = xStopa;
            T1 = T1a;
            T2 = T2a;
            tx = txa;
            xPs = xPsa;
            yPs = yPsa;
            spacialT1 = spacialT1a;
            spacialT2 = spacialT2a;
            Tmax1 = Tmax1a;
            Tmax2 = Tmax2a;
            Trms1 = Trms1a;
            Trms2 = Trms2a;
            data.trayPosition = 1;
        else
            yStart = yStartb;
            yStop = yStopb;
            xStart = xStartb;
            xStop = xStopb;
            T1 = T1b;
            T2 = T2b;
            tx = txb;
            xPs = xPsb;
            yPs = yPsb;
            spacialT1 = spacialT1b;
            spacialT2 = spacialT2b;
            Tmax1 = Tmax1b;
            Tmax2 = Tmax2b;
            Trms1 = Trms1b;
            Trms2 = Trms2b;
            data.trayPosition = 2;
        end

        if options.Tsims || options.all
            data.robot = robot;
            data.xStart = xStart;
            data.xStop = xStop;
            data.yStart = yStart;
            data.yStop = yStop;
            data.T1 = T1;
            data.T2 = T2;
            data.tx = tx;
            data.xPs = xPs;
            data.yPs = yPs;
        end

        if options.Tspecs || options.all
            Tspec1 = nan(nStepsY, nStepsX);
            Tspec2 = Tspec1;

            if options.rotatedWS
                for i = 1:length(tx)
                    Tspec1(yPs{i}(1), :) = spacialT1{i}(1:nStepsX);
                    Tspec2(yPs{i}(1), :) = spacialT2{i}(1:nStepsX);
                end
            else
                for i = 1:length(tx)
                    Tspec1(:, xPs{i}(1)) = spacialT1{i}(1:nStepsY);
                    Tspec2(:, xPs{i}(1)) = spacialT2{i}(1:nStepsY);
                end
            end

            data.Tspec1 = Tspec1;
            data.Tspec2 = Tspec2;
        end
        
        if options.Tmax || options.all
            data.Tmax1 = Tmax1;
            data.Tmax2 = Tmax2;

            data.Tmax1a = Tmax1a;
            data.Tmax2a = Tmax2a;
            % if options.optimizeTrayPosition
                data.Tmax1b = Tmax1b;
                data.Tmax2b = Tmax2b; 
            % end
        end
        
        if options.Trmsmax || options.all
            data.Trmsmax1 = calculateGlobalIndex(Trms1, "Trms");
            data.Trmsmax2 = calculateGlobalIndex(Trms2, "Trms");
        end

    end
end

warning('on','all')