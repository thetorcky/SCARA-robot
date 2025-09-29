function matrixStat = calculateMatrixStat(localIndex, options)
% calculateGlobalIndex - Calculates a statistic for the elements of the
% provided matrix
%  
% matrixStat = calculateGlobalIndex(localIndex, options)
% INPUTS:   localIndex  = nStepsY x nStepsX matrix, containing a local
%                         performance index 
%           options     = name-value pairs cointaining plotting preferences
%           - type              = string containing the global index 
%                                 calculation method, between avg, min and
%                                 max. Default: avg
% OUTPUTS:  matrixStat  = the global performance index, calculated
%                         according to the selected type.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    localIndex double
    options.type string = "avg"
    options.boundIDs
end

nPages = size(localIndex, 3);
matrixStat = nan(1, nPages);

for i = 1:nPages
    switch options.type
        case "avg"
            matrixStat(i) = mean(localIndex(:,:,i), "all", "omitnan");
        case "min"
            matrixStat(i) = min(localIndex(:,:,i), [], "all", "omitnan");
        case "max"
            matrixStat(i) = max(localIndex(:,:,i), [], "all", "omitnan");
    end
end