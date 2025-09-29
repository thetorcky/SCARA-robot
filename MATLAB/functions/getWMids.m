function WMids = getWMids(rectangleWMs)
% getWMids - Converts the list of slice working modes, from being expressed
% as 1s in on a line if the respective WM is usable, to columns of the
% actual ids (1-4) of the usable WMs. Note: the first line contains the
% number of usable WMs
%  
% WMids = getWMids(rectangleWMs)
% INPUTS:   WMs                 = nWMs+1 x width matrix containing the 
%                                 possible working modes for each slice 
%                                 that result in the specified workspace 
%                                 rectangle, specified as ones on the line
%                                 of the corresponding line.
% OUTPUTS:  WMids               = nWMs+1 x width matrix containing the 
%                                 possible working modes for each slice 
%                                 that result in the specified workspace 
%                                 rectangle, specified the ids of the WMs.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

WMids = zeros(size(rectangleWMs, 1), size(rectangleWMs, 2));
for i = 1:size(rectangleWMs, 2)
    for wm = 2:size(rectangleWMs, 1)
        if rectangleWMs(wm, i)
            WMids(1, i) = WMids(1, i) + 1;
            WMids(WMids(1, i) + 1, i) = wm - 1;
        end
    end
end