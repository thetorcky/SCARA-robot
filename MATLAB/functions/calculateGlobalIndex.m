function globalIndex = calculateGlobalIndex(localIndex, name, options)
% calculateGlobalIndex - Calculates the global performance index for a
% provided matrix of local performance data.
%  
% globalIndex = calculateGlobalIndex(localIndex, options)
% INPUTS:   localIndex  = nStepsY x nStepsX matrix, containing a local
%                         performance index 
%           name        = name of the index to be calculated
%           options     = name-value pairs cointaining plotting preferences
%           - bounds            = optional boundry IDs for calculating the
%                                 index for a subset of the provided data.
% OUTPUTS:  globalIndex = the global performance index, calculated
%                         according to the selected type.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    localIndex double
    name string
    options.boundIDs
end

if isfield(options, "boundIDs") && ~isnan(options.boundIDs.yTopID)
    localIndex = localIndex(options.boundIDs.yTopID:options.boundIDs.yBotID, options.boundIDs.xLeftID:options.boundIDs.xRightID, :);
end

switch name
    case "GCI"
        globalIndex = calculateMatrixStat(localIndex);
    case "GMPE"
        globalIndex = calculateMatrixStat(localIndex, type = "max");
    case "Gtau"
        globalIndex = calculateMatrixStat(localIndex, type = "max");
    case "Tmax"
        globalIndex = calculateMatrixStat(abs(localIndex), type = "max");
    case "Trms"
        globalIndex = calculateMatrixStat(abs(localIndex), type = "max");
    case "elbowRange"
        globalIndex = calculateMatrixStat(localIndex, type = "max");
    otherwise
        warning("calculateGlobalIndex: Unknown performance index!");
        return
end
