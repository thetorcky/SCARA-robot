function LSI = calculateLocalStiffnessIndex (JJ)
% calculateTorqueTransmissibility - Calculates the local stiffness index 
% for all poses in the workspace. Defined as the largest displacement that
% can occur from an equal force on the end effector applied in an arbitrary
% direction. Calculated as the maximum singular value of the compliance
% matrix: inv(transpose(J)*J) (Liu et al, 2006):
%
% D = inv(K) * tau, where tau = 1
% K = J^T * I * J
%  
% LSI = calculateLocalStiffnessIndex (JJ)
% INPUTS:   JJ           = 4-D array containing the Jacobian of the robot
%
% OUTPUTS:  LSI          = matrix containing the local stiffness index at 
%                          each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

JJtranspose = pagetranspose(JJ);

S = pagesvd(pageinv(pagemtimes(JJtranspose, JJ)));

LSI(:,:) = S(1,1,:,:);