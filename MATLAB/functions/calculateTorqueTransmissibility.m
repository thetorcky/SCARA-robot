function LTT = calculateTorqueTransmissibility (JJ)
% calculateTorqueTransmissibility - Calculates the local torque 
% transmisibility for all poses in the workspace
%  
% LTT = calculateTorqueTransmissibility (JJ)
% INPUTS:   JJ           = 4-D array containing the Jacobian of the robot
%
% OUTPUTS:  LTT          = matrix containing the local torque 
%                          transmisibility at each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

JJtinv = pageinv(pagetranspose(JJ));

S = pagesvd(JJtinv);

% LTT(:,:) = S(1,1,:,:);
LTT(:,:) = S(1,1,:,:) ./ S(2,1,:,:);
