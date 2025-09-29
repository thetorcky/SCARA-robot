function [a, b, w, selectedData] = plotSelectedRegion (a, b, w, data, operator, limit)
% WIP - doesn't plot anything yet
% plotSelectedRegion - Selects the region of an atlas containing data with
% that satisfies a performance limit.
%  
% [a, b, w, selectedData] = plotSelectedRegion (a, b, w, data, operator, limit)
% INPUTS:   a           = driven link length mesh points
%           b           = floating link length mesh points
%           w           = half of the base link length mesh points
%           data        = 2D matrix containing a global performance index
%                         data for all designs in the design space (= for
%                         each a-b pair)
%           operator    = mathematical operator used to select the desired
%                         performance limits
%           limit       = numerical value representing the percentage of
%                         the atlas that is below the requirement and is
%                         thus removed
% OUTPUTS:  a           = driven link length mesh points
%           b           = floating link length mesh points
%           w           = half of the base link length mesh points
%           data        = 2D matrix containing the normalized performance
%                         values that satisfy the requirement
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

selectedData = data;
selectedData = nanMatrixOutliers(selectedData);
selectedData(isnan(a)) = nan;
selectedData(isnan(b)) = nan;

for i = 1:length(operator)
    op = operator(i);
    li = limit(i);
    switch op
        case "<"
            selectedData = (max(selectedData, [], "all") - selectedData) ./ (max(selectedData, [], "all") - min(selectedData, [], "all"));
            selectedData(selectedData <= li) = nan;
        
        case ">"
            selectedData = (selectedData - min(selectedData, [], "all")) ./ (max(selectedData, [], "all") - min(selectedData, [], "all"));
            selectedData(selectedData <= li) = nan;
    
        otherwise
            disp("selectRegion Error: unknown operator");
    end
end

% k = boundary(a(~isnan(selectedData)), b(~isnan(selectedData)), 1);

hold on
% axSurf = surf(a-0.05*1.5, b-0.05*1.5, w-0.07*1.5, selectedData, 'edgecolor', 'none');     % ./max(selectedData, [], "all")

% a = a(~isnan(selectedData));
% b = b(~isnan(selectedData));
% w = w(~isnan(selectedData));
% ax = plot3(a(k), b(k), w(k), "-.", 'linewidth', 2);
