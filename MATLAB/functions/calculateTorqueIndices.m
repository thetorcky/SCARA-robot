function [tauAcc1, tauVel1, tauAcc2, tauVel2, Metilda, Hetilda1, Hetilda2] = calculateTorqueIndices(robot, xP, yP, q, options)
% calculateTorqueIndices - Calculates the local torque indices and
% separated dynamics matrices for all poses in the specified workspace
%  
% [tauAcc1, tauVel1, tauAcc2, tauVel2, Metilda, Hetilda1, Hetilda2] = ...
%           calculateTorqueIndices(robot, xP, yP, q, options)
% INPUTS:   robot       = robot struct containing the mechanical properties
%                         required for dynamics calculations
%           xP          = nSteps x nSteps matrix of X coordinates
%           yP          = nSteps x nSteps matrix of Y coordinates
%           q           = nSteps x nSteps x 4 paged matrix containing the
%                         dependant joint coordinates at each pose
%           options     = name-value pairs cointaining plotting preferences
%           - bounds            = optional boundry IDs for calculating the
%                                 index for a subset of the provided data.
%           - driveAtElbows         = drive position selection. 1 for
%                                     driving at elbows relative to the 
%                                     ground. 2 for driving at the elbows,
%                                     between the links. 0 for driving at
%                                     the shoulders.
% OUTPUTS:  tauAcc1     = local acceleration index for motor 1
%           tauVel1     = local velocity index for motor 1
%           tauAcc2     = local acceleration index for motor 2
%           tauVel2     = local velocity index for motor 2
%           Metilda     = nSteps x nSteps cell array of the Metilda matrix
%                         for each pose
%           Hetilda1    = nSteps x nSteps cell array of the Hetilda1 matrix
%                         for each pose, the first row of Hetilda
%           Hetilda2    = nSteps x nSteps cell array of the Hetilda2 matrix
%                         for each pose, the second row of Hetilda
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    robot struct
    xP double
    yP double
    q double
    options.boundIDs
    options.driveAtElbows (1,1) int8 = 0
end

tauAcc1 = nan(size(yP,1), size(xP, 2));
tauAcc2 = tauAcc1;
tauVel1 = tauAcc1;
tauVel2 = tauAcc1;

Metilda = cell(size(yP,1), size(xP, 2));
Hetilda1 = Metilda;
Hetilda2 = Metilda;

if isfield(options, "boundIDs") && ~isnan(options.boundIDs.yTopID)
    iMin = options.boundIDs.yTopID;
    iMax = options.boundIDs.yBotID;
    jMin = options.boundIDs.xLeftID;
    jMax = options.boundIDs.xRightID;
else
    iMin = 1;
    iMax = size(yP, 1);
    jMin = 1;
    jMax = size(xP, 2);
end

A = robot.A;
B = robot.B;
W = robot.W;

for i = iMin:iMax
    for j = jMin:jMax
        qi(:, :) = q(i, j, :);

        [M, ~, ~, ~, Chat] = dynamicsAssembleMatrices(robot, qi, qi*0);

        T = calculateJacobianExtended(A, B, W, qi);
        TT = transpose(T);
    
        delT = calculateDelT(A, B, W, qi);
    
        if options.driveAtElbows == 1
            JJ = [T(2,:); T(4,:)];
        elseif options.driveAtElbows == 2
            JJ = [T(2,:); T(4,:)] - [T(1,:); T(3,:)];
        else
            JJ = [T(1,:); T(3,:)];
        end
        
        Ce1 = inv(transpose(JJ)) * TT * M;
        Ce2 = inv(transpose(JJ)) * TT;
        Metildaij = inv(transpose(JJ)) * TT * M * T;
        Hetilda1ij = calculateHetildaj(1, T, TT, delT, Ce1, Ce2, Chat);
        Hetilda2ij = calculateHetildaj(2, T, TT, delT, Ce1, Ce2, Chat);
    
        tauAcc1(i, j) = max(svd(Metildaij(1, :)));
        tauVel1(i, j) = max(svd(Hetilda1ij));
        tauAcc2(i, j) = max(svd(Metildaij(2, :)));
        tauVel2(i, j) = max(svd(Hetilda2ij));

        Metilda{i, j} = Metildaij;
        Hetilda1{i, j} = Hetilda1ij;
        Hetilda2{i, j} = Hetilda2ij;
    end
end
