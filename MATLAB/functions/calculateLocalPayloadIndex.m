function LPI = calculateLocalPayloadIndex (JJ)
% calculateTorqueTransmissibility - Calculates the local payload index 
% for all poses in the workspace. Defined as the base line force that can
% be applied to the end effector and withstood in any direction. Calculated
% as the minimum singular value of transpose(J) (Liu et al, 2006):
%
% tau = J^T * f, where f = 1
%  
% LTT = calculateLocalPayloadIndex (JJ)
% INPUTS:   JJ           = 4-D array containing the Jacobian of the robot
%
% OUTPUTS:  LPI          = matrix containing the local payload index at 
%                          each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

JJtranspose = pagetranspose(JJ);

S = pagesvd(JJtranspose);

LPI(:,:) = S(2,1,:,:);