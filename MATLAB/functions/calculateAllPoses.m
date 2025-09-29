function results = calculateAllPoses(a, b, w, nStepsX, nStepsY, stepSizeX, stepSizeY, wm, am, options)
% calculateAllPoses - Calculates the local performance indices for all
% poses in the workspace of the robot, in a given working mode
%  
% results = calculateAllPoses(a, b, w, nSteps, stepSizeX, stepSizeY)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           nStepsX     = number of steps in the X direction of 
%                         the discretized workspace
%           nStepsY     = number of steps in the Y direction of 
%                         the discretized workspace
%           stepSizeX   = step size in the X direction
%           stepSizeY   = step size in the Y direction
%           wm[1x2]     = vector containing the branch indices for the two
%                         arms: wm = [delta1, delta2]
%           am          = assembly mode branch index corresponding to the
%                         selected AM: +1 or -1
%           options     = name-value pairs cointaining plotting preferences
%           - senseAtElbows         = calculate the MPE based on encoders
%                                     positioned at the elbow joints,
%                                     instead of the shoulders
%           - senseAtEither         = calculate the MPE based on encoders
%                                     positioned at the elbow joints or
%                                     at the shoulders
%           - driveAtElbows         = drive position selection. 1 for
%                                     driving at elbows relative to the 
%                                     ground. 2 for driving at the elbows,
%                                     between the links. 0 for driving at
%                                     the shoulders.
% OUTPUTS:  results     = struct containting matrices with the local
%                         performance indices for each pose in the workspace.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    a double
    b double
    w double
    nStepsX double
    nStepsY double
    stepSizeX double
    stepSizeY double
    wm (1,2) double
    am double
    options.senseAtElbows (1,1) logical = false
    options.senseAtEither (1,1) logical = false
    options.driveAtElbows (1,1) int8 = 0
end

if options.driveAtElbows == 1
    error("driveAtElbows == 1 not properly implemented")
end

i   = 1:nStepsY;
j   = 1:nStepsX;

xP  = repmat(stepSizeX*(j-fix(nStepsX/2)-1),nStepsY,1);
yP  = repmat(-stepSizeY*(i'-fix(nStepsY/2)-1),1, nStepsX);

[theta1, theta2] = IK(xP, yP, a, b, w, wm);
[beta1, beta2] = calculateFloatingAngles(a, b, w, xP, yP, theta1, theta2);
[theta1_senseAtElbows, theta2_senseAtElbows] = IK_senseAtElbows(xP, yP, a, b, w, wm);

% Remove other AM from the reachable workspace

[xPthisAM, yPthisAM, ~, ~, ~, ~] = FK(theta1, theta2, a, b, w, am);
maskX = zeros(size(theta1, 1), size(theta1, 2));
maskY = maskX;
maskX(abs(xP - xPthisAM) < 1e-5) = 1;
maskY(abs(yP - yPthisAM) < 1e-5) = 1;

mask = maskX & maskY;
theta1(~mask) = NaN;
theta2(~mask) = NaN;

[xPthisAM, yPthisAM, ~, ~, ~, ~] = FK_betas(beta1, beta2, a, b, w, am);
maskX = zeros(size(theta1, 1), size(theta1, 2));
maskY = maskX;
maskX(abs(xP - xPthisAM) < 1e-5) = 1;
maskY(abs(yP - yPthisAM) < 1e-5) = 1;

mask = maskX & maskY;
beta1(~mask) = NaN;
beta2(~mask) = NaN;

[xPthisAM, yPthisAM, ~, ~, ~, ~] = FK_senseAtElbows(theta1_senseAtElbows, theta2_senseAtElbows, a, b, w, am);
maskX = zeros(size(theta1, 1), size(theta1, 2));
maskY = maskX;
maskX(abs(xP - xPthisAM) < 1e-5) = 1;
maskY(abs(yP - yPthisAM) < 1e-5) = 1;

mask = maskX & maskY;
theta1_senseAtElbows(~mask) = NaN;
theta2_senseAtElbows(~mask) = NaN;

%% Jacobian
if options.senseAtElbows || options.senseAtEither
    JJsensingAtElbows = calculateJacobian_senseAtElbows(a, b, w, xP, yP, theta1_senseAtElbows, theta2_senseAtElbows);
    if options.senseAtElbows
        JJsensing = JJsensingAtElbows;
    end
end
if ~options.senseAtElbows || options.senseAtEither
    JJsensingAtShoulders = calculateJacobianThetas(a, b, w, xP, yP, theta1, theta2);
    if ~options.senseAtElbows
        JJsensing = JJsensingAtShoulders;
    end
end

if options.driveAtElbows == 1
    JJdriving = calculateJacobianBetas(a, b, w, xP, yP, beta1, beta2);
elseif options.driveAtElbows == 2
    JJdriving = calculateJacobian_senseAtElbows(a, b, w, xP, yP, theta1_senseAtElbows, theta2_senseAtElbows);
else
    JJdriving = calculateJacobianThetas(a, b, w, xP, yP, theta1, theta2);
end

%% LCI
S = pagesvd(JJsensing);
LCIobservability(:,:) = S(2,1,:,:) ./ S(1,1,:,:);
S = pagesvd(JJdriving);
LCIcontrollability(:,:) = S(2,1,:,:) ./ S(1,1,:,:);

LCI = sqrt(LCIobservability .* LCIcontrollability);

% plotPerformanceIndex (xP, yP, "LCI", LCIobservability, "theoreticalWorkspace", [a; b; w], onlyAboveX=1);
% plotPerformanceIndex (xP, yP, "LCI", LCIcontrollability, "theoreticalWorkspace", [a; b; w], onlyAboveX=1);
% plotPerformanceIndex (xP, yP, "LCI", LCI, "theoreticalWorkspace", [a; b; w], onlyAboveX=1);

%% MPE
% encRepeatability = 0.000075;
encRepeatability = 0.000023;
MPE = calculateMaximumPositioningError (JJsensing, encRepeatability);

if options.senseAtEither
    if options.senseAtElbows
        MPEsenseAtElbows = MPE;
        MPEsenseAtShoulders = calculateMaximumPositioningError (JJsensingAtShoulders, encRepeatability);
    else
        MPEsenseAtShoulders = MPE;
        MPEsenseAtElbows = calculateMaximumPositioningError (JJsensingAtElbows, encRepeatability);
    end
end

%% LPI
% LPI = calculateLocalPayloadIndex (JJ);

%% LTT
% LTT = calculateTorqueTransmissibility (JJ);

%% LSI
% LSI = calculateLocalStiffnessIndex (JJ);

%% LMI
% LMI = calculateLocalManipulabilityIndex (JJ);

%% Results
results.xP = xP;
results.yP = yP;

results.theta1 = theta1;
results.theta2 = theta2;

[beta1, beta2] = calculateFloatingAngles(a, b, w, xP, yP, theta1, theta2);
results.beta1 = beta1;
results.beta2 = beta2;

results.LCI = LCI;

results.MPE = MPE;

if options.senseAtEither
    results.MPEsenseAtShoulders = MPEsenseAtShoulders;
    results.MPEsenseAtElbows = MPEsenseAtElbows;
end

% results.LPI = LPI;
% 
% results.LTT = LTT;
% 
% results.LSI = LSI;
% 
% results.LMI = LMI;

