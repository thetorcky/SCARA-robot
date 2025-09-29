function cid = plotPerformanceIndex (xP, yP, name, data, options)
% plotPerformanceIndex - Plots one local performance index for a given
% robot. If the data matrix contains multiple pages (in the 3rd dimension,
% then each page is plotted independently (useful for multiple working
% modes).
%  
% cid = plotPerformanceIndex (xP, yP, name, data, options)
% INPUTS:   xP          = nSteps x nSteps matrix of X coordinates
%           yP          = nSteps x nSteps matrix of Y coordinates
%           name        = index identifier. Must match the ones defined
%                         below
%           data        = nSteps x nSteps x K matrix of data points to be
%                         plotted. If K > 1, each page in the 3rd dimension
%                         of the data matrix is plotted independently.
%           options     = name-value pairs cointaining plotting preferences
%           - plotSizeX             = plot size in the X-direction
%           - plotSizeY             = plot size in the Y-direction
%           - levels                = parsed to the "levels" parameter of 
%                                     contourf()
%           - ShowText              = parsed to the "ShowText" parameter of 
%                                     contourf()
%           - fontsize              = font size of text in the plot
%           - grid                  = boolean to control the grid display
%           - theoreticalWorkspace  = 3 x 1 vector of robot link lengths to
%                                     be used to plot the theoretical
%                                     reachable workspace. If empty, then
%                                     the theoretical workspace is not
%                                     drawn.
%           - onlyAboveX            = boolean to control plotting of the
%                                     poses below the X axis.
%           - workspaceBounds       = struct containing the IDs of the top 
%                                     row, bottom row, left column and 
%                                     right column of the workspace 
%                                     rectangle bounds. If empty, then no 
%                                     workspace is drawn.
%           - workspaceSlices       = 1 x width vector containing the IDs
%                                     of the working modes for each slice
%                                     of the workspace. If empty, then the
%                                     slices are not drawn.
% OUTPUTS:  cid         = identifier of the created plot
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    xP double
    yP double
    name string
    data double
    options.plotSizeX (1,1) double = 6
    options.plotSizeY (1,1) double = 6
    options.levels double
    options.ShowText (1,1) logical = false
    options.fontsize (1,1) double = 14
    options.grid (1,1) logical = true
    options.theoreticalWorkspace (3,1) double
    options.onlyAboveX (1,1) logical = false
    options.workspaceBounds  
    options.workspaceSlices cell
end

flipColorbar = false;
logScale = false;
colorbarColormap = "parula";
colorbarLimits = false;

% Index lookup
switch name

    case "misc"
        plotTitle = 'Local performance plot';
        colorbarLabel = "[???]";

    case "theta1"
        plotTitle = 'Motor angle \theta_1';
        colorbarLabel = "\theta_1 [°]";

    case "theta2"
        plotTitle = 'Motor angle \theta_2';
        colorbarLabel = "\theta_2 [°]";

    case "beta1"
        plotTitle = 'Floating link angle \beta_1';
        colorbarLabel = "\beta_1 [°]";

    case "beta2"
        plotTitle = 'Floating link angle \beta_2';
        colorbarLabel = "\beta_2 [°]";

    case "phi1"
        plotTitle = 'Elbow angle \phi_1';
        colorbarLabel = "\phi_1 [°]";
        flipColorbar = true;

    case "phi2"
        plotTitle = 'Elbow angle \phi_2';
        colorbarLabel = "\phi_2 [°]";
        flipColorbar = true;

    case "LCI"
        plotTitle = 'Local conditioning index LCI';
        colorbarLabel = "LCI [-]";

    case "MPE"
        plotTitle = 'Maximum positioning error MPE';
        colorbarLabel = "MPE [μm]";
        data = data*1e6;
        flipColorbar = true;
        logScale = true;

    case "LPI"
        plotTitle = 'Local payload index LPI';
        colorbarLabel = "LPI [-]";
        % logScale = true;

    case "LTT"
        plotTitle = 'Local torque transmisibility LTT';
        colorbarLabel = "LTT [-]";
        flipColorbar = true;
        logScale = true;

    case "LSI"
        plotTitle = 'Local stiffness index LSI';
        colorbarLabel = "LSI [-]";
        flipColorbar = true;
        logScale = true;

    case "LMI"
        plotTitle = 'Local manipulability index LMI';
        colorbarLabel = "LMI [-]";
        % flipColorbar = true;
        % logScale = true;

    case "tauAcc1"
        plotTitle = 'Local torque index tauAcc1';
        colorbarLabel = "tauAcc1 [Nm/(m/s^2) = kgm]";
        flipColorbar = true;
        % logScale = true;

    case "tauAcc2"
        plotTitle = 'Local torque index tauAcc2';
        colorbarLabel = "tauAcc2 [Nm/(m/s^2) = kgm]";
        flipColorbar = true;
        % logScale = true;

    case "tauVel1"
        plotTitle = 'Local torque index tauVel1';
        colorbarLabel = "tauVel1 [Nm/(m^2/s^2) = kg]";
        flipColorbar = true;
        % logScale = true;

    case "tauVel2"
        plotTitle = 'Local torque index tauVel2';
        colorbarLabel = "tauVel2 [Nm/(m^2/s^2) = kg]";
        flipColorbar = true;
        % logScale = true;

    case "Tspec1"
        plotTitle = 'Spacial specification torque Tspec1';
        colorbarLabel = "Tspec1 [Nm]";
        flipColorbar = true;
        colorbarColormap = jet(256);
        colorbarColormap = flipud(colorbarColormap(64:256, :));
        colorbarLimits = true;

    case "Tspec2"
        plotTitle = 'Spacial specification torque Tspec2';
        colorbarLabel = "Tspec2 [Nm]";
        flipColorbar = true;
        colorbarColormap = jet(256);
        colorbarColormap = flipud(colorbarColormap(64:256, :));
        colorbarLimits = true;

    otherwise
        disp("plotPerformanceIndex: Unknown performance index!");
        return
end

if isfield(options, "workspaceBounds") && yP(options.workspaceBounds.yBotID, 1) < 0 
    options.onlyAboveX = false;
end

if options.onlyAboveX
    data(fix(size(data,1)/2)+2:size(data,1),:,:) = [];
    xP(fix(size(xP,1)/2)+2:size(xP,1),:,:) = [];
    yP(fix(size(yP,1)/2)+2:size(yP,1),:,:) = [];
end

nPlots = size(data, 3);

for i = 1:nPlots
    figure();
    set(gcf,'renderer','painters');

    if isfield(options, "levels")
        [~, cid] = contourf(xP, yP, data(:,:,i), options.levels, "ShowText", false);
    else
        [~, cid] = contourf(xP, yP, data(:,:,i), "ShowText", false);
    end

    % plotInscribedRectangle(xP, yP, CoordIDs);
    if isfield(options, "theoreticalWorkspace")
        plotTheoreticalWorkspace(options.theoreticalWorkspace(1), options.theoreticalWorkspace(2), options.theoreticalWorkspace(3));
    end

    axis equal
    c = colorbar;
    c.Label.String = colorbarLabel;
    colormap(colorbarColormap);
    if flipColorbar
        colormap(flipud(colormap));
    end
    if logScale
        set(gca,'ColorScale','log')
    end
    if colorbarLimits
        clim([-max(abs(data(:))), max(abs(data(:)))]);
    end

    if name ~= "misc"
        xlim([-options.plotSizeX/2 options.plotSizeX/2]);
        if options.onlyAboveX 
            ylim([0 options.plotSizeY/2]);
        else
            ylim([-options.plotSizeY/2 options.plotSizeY/2]);
        end
    end
    
    if(nPlots > 1)
        WMcolors = {[1 0 0], [0 0.4470 0.7410], [0.4660 0.6740 0.1880], [0.9290 0.6940 0.1250]};
        plotTitlei = sprintf('%s - \\color[rgb]{%f,%f,%f}WM %d', plotTitle, WMcolors{i}(1), WMcolors{i}(2), WMcolors{i}(3), i);
        title(plotTitlei, 'Interpreter', 'tex');
    else
        title(plotTitle);
    end
    
    if name ~= "misc"
        xlabel("x (normalized) [-]");
        ylabel("y (normalized) [-]");
    end
    
    fontsize(options.fontsize,"points"); 
    
    if options.grid
        grid on
    end

    if isfield(options, "workspaceBounds")
        plotRectangle(xP, yP, options.workspaceBounds);
        if isfield(options, "workspaceSlices")
            plotWSSlices(xP, yP, options.workspaceBounds, options.workspaceSlices);
        end
    end
end
