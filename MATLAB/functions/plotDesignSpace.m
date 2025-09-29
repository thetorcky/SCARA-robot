function plotDesignSpace (varargin)
% plotDesignSpace - Plots data over the nondimensional design space
% trapezoid for symmetric 5-bar SCARA robots.
%  
% plotDesignSpace(a, b, w) - Creates a trapezoid plot of the
% nondimensional design space and highlights the positioning of the robot
% design defined by link lengths a, b and w.
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
% 
% plotDesignSpace(designSpace, perfAtlases) - Plots the color contour plot
% for all performance atlases provided in perfAtlases.
% INPUTS:   designSpace = coordinates of the points in the design space
%                         discretization used to calculate the performance 
%                         atlases.
%           perfAtlases = matrices containing the data points to be ploted
%                         in the atlases.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

switch length(varargin)
    case 3          % Plot placement of a configuration in the design space
        a = varargin{1};
        b = varargin{2};
        w = varargin{3};

    case 2          % Plot performance atlases over design space
        designSpace = varargin{1};
        perfAtlases = varargin{2};

    otherwise
        disp("plotDesignSpace: Insufficient arguments!");
        return
end

switch length(varargin)
    case 3          % Plot placement of a configuration in the design space
        [aa, bb] = meshgrid(0:0.05:3);
        ww = 3 - aa - bb;
        ww(ww>1.5) = nan;
        ww(ww<0) = nan;
        
        plotTrapezoid();
        hold on
        scatter3(aa, bb, ww, 10, 'k', 'x');
        scatter3(a, b, w, 80, 'r', 'c', 'filled');
        hold off
        title("Normalized design space");
        aH = ancestor(gca,'axes');
        set(aH,'XLim',[0 3]);
        set(aH,'YLim',[0 3]);
        set(aH,'ZLim',[0 1.5]);
        fontsize(16,"points");

    case 2          % Plot performance atlases over design space
        disp("plotDesignSpace: Not implemented!");
        return

    otherwise
        disp("plotDesignSpace: Insufficient arguments!");
        return
end
