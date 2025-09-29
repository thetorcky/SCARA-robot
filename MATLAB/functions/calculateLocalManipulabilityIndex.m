function LMI = calculateLocalManipulabilityIndex (JJ)
% calculateLocalManipulabilityIndex - Calculates the local manipulability
% index. Defined as determinant of the Jacobian matrix (Demjen et al, 2023)
%
% LMI = calculateLocalManipulabilityIndex (JJ)
% INPUTS:   JJ           = 4-D array containing the Jacobian of the robot
%
% OUTPUTS:  LMI          = matrix containing the local manipulability index 
%                          at each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

LMI = abs(pagedet(pageinv(JJ)));