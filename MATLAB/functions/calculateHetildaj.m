function Hetildaj = calculateHetildaj(j, T, TT, delT, Ce1, Ce2, Chat)
% calculateHetildaj - Calculates the He tilda term for arm j of the robot.
% Where the Hetilda term can be used to determine the torque contribution
% of the centrifugal/coriolis (velocity related) terms:
%
% tau_velocity(j) = qp' * Hetildaj * qp
% 
% Hetildaj = calculateHetildaj(j, T, TT, delT, Ce1, Ce2, Chat)
% INPUTS:   j           = ID of the arm (1 or 2). Corresponds to the row
%                         number in the dynamics matrices.
%           T           = 4x2 coordinate transformation matrix
%           TT          = transpose of the trasformation matrix
%           delT        = 4x2 cell array of 1x4 partial derivatives of the
%                         transformation matrix
%           Ce1         = 2x4 matrix containing terms related to the
%                         derivative of T
%           Ce2         = 2x4 matrix containing terms related to the
%                         C matrix
%           Chat        = 4x1 cell array of 4x4 matrices representing the
%                         diagonal term representation of the C matrix
% OUTPUTS:  Hetildaj    = 2x2 matrix representing the contribution of the
%                         velocity terms to the dynamics.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

Hetildaj = zeros(2,2);
for k = 1:4
    Hkbar = TT * [delT{k,1}' delT{k,2}'];
    Hetildaj = Hetildaj + (Ce1(j,k) * Hkbar + Ce2(j,k)*TT*Chat{k}*T);
end