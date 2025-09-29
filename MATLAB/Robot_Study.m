% Robot_Study - Calculates and plots the performance characteristics of a
% particular 5-bar robot design, based on the ratios of its link lengths.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

clearvars; clc;close all;
addpath(genpath('functions\'))

%% Link lengths
tic
% From example
selectExample = 20;
AExamples = [1.1000    1.3500    2.0000    2.0000    1.4000    0.5500    0.6000    0.2000    0.7000    1.0000    0.8500    1.0000    1.2000    1.5000    1.9000    0.1940    0.2880    0.9500    0.3500    1.3000    0.2880    0.2880    230.00];
BExamples = [0.5500    0.6000    0.2000    0.7000    1.0000    1.1000    1.3500    2.0000    2.0000    1.4000    0.8500    1.0000    1.2000    1.5000    2.1000    0.2940    0.2880    1.7000    1.7000    1.7000    0.2880    0.2880    230.00];
WExamples = [1.3500    1.0500    0.8000    0.3000    0.6000    1.3500    1.0500    0.8000    0.3000    0.6000    1.3000    1.0000    0.6000    0.0000    0.7035    0.0770    0.0600    0.3500    0.9500    0.0000    0.0600    0.0600    137.50];
txtExampl = ["Ia"      "IIa"     "IIIa"    "IVa"     "Va"      "Ib"      "IIb"     "IIIb"    "IVb"     "Vb"      "Imid"    "IImid"   "Vmid"    "w=0"     "Hunag"   "Demjen"  "Dynam"   "LS1"     "LS2"     "LS3"     "LS4.1"   "LS4.2"   "DexTAR"];
A = AExamples(selectExample);
B = BExamples(selectExample);
W = WExamples(selectExample);
robotTitle = txtExampl(selectExample);

% From user input
A = 1.025;                  % [m]
B = 1.625;                 % [m]
W = 0.35;                % [m]
robotTitle = "User input";

fprintf("Starting study for robot %s.\n", robotTitle);

D = (A+B+W)/3;

a = A/D;
b = B/D;
w = W/D;

s = 2*a/sqrt(3) + w/sqrt(3);
t = w;

% plotDesignSpace(a,b,w);

%% Calculate robot

onlyAboveX = true;
optimizeGCI = false;
rotatedWS = false;
senseAtElbows = false;
senseAtEither = true;
optimizeTrayPosition = false;
driveAtElbows = 0;
phiLimit = 90 + 90;

desiredWorkspaceWidth = 0.400;
desiredAspectTan = 256/400;

LCIlimit = 0.25;
nStepsX = 101;

setpoint.vMax = 3;
setpoint.aMax = 80;
setpoint.jMax = 4000;
setpoint.dwellTime = 0.1;

WMs = {[1, 1], [-1, 1], [1, -1], [-1, -1]};
% WMs = WMs(1);
am = 1;

if rotatedWS
    desiredAspectTan = 1 / desiredAspectTan;
    nStepsX = round(nStepsX * desiredAspectTan);
end

d = calculateRobot(a, b, w, WMs, am, nStepsX, desiredWorkspaceWidth, desiredAspectTan, LCIlimit, setpoint, all = true, ...
    onlyAboveX = onlyAboveX, optimizeGCI = optimizeGCI, rotatedWS = rotatedWS, senseAtElbows = senseAtElbows, senseAtEither = senseAtEither, optimizeTrayPosition = optimizeTrayPosition, driveAtElbows = driveAtElbows, phiLimit = phiLimit);

fprintf("\nA = %f\nB = %f\nW = %f\n\n", d.robot.A, d.robot.B, d.robot.W);

%% Plot thetas
% plotPerformanceIndex (d.xP, d.yP, "theta1", rad2deg(d.theta1stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
% plotPerformanceIndex (d.xP, d.yP, "theta2", rad2deg(d.theta2stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);

%% Plot betas
% plotPerformanceIndex (d.xP, d.yP, "beta1", rad2deg(d.beta1stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
% plotPerformanceIndex (d.xP, d.yP, "beta2", rad2deg(d.beta2stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);

%% Plot phis
% plotPerformanceIndex (d.xP, d.yP, "phi1", rad2deg(d.phi1stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
% plotPerformanceIndex (d.xP, d.yP, "phi2", rad2deg(d.phi2stitched), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);

%% Plot local conditioning index
plotPerformanceIndex (d.xP, d.yP, "LCI", d.LCInan, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, workspaceSlices={d.allRectangleWMs, rotatedWS});
plotPerformanceIndex (d.xP, d.yP, "LCI", d.LCIstitched, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, workspaceSlices={d.rectangleWMs, rotatedWS});
fprintf("GCI = %f\n\n", d.GCI);

%% Plot maximum positioning error
% plotPerformanceIndex (d.xP, d.yP, "MPE", nanMatrixOutliers(d.MPE), "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
plotPerformanceIndex (d.xP, d.yP, "MPE", d.MPEstitched, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
fprintf("GMPE = %f\n", d.GMPE*1e6);
if senseAtEither
    fprintf("GMPEsenseAtShoulders = %f;     GMPEsenseAtElbows = %f\n\n", d.GMPEsenseAtShoulders*1e6, d.GMPEsenseAtElbows*1e6);
end

%% Torque indices
plotPerformanceIndex (d.xP, d.yP, "tauAcc1", d.tauAcc1, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, levels=15);
plotPerformanceIndex (d.xP, d.yP, "tauVel1", d.tauVel1, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, levels=15);
plotPerformanceIndex (d.xP, d.yP, "tauAcc2", d.tauAcc2, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, levels=15);
plotPerformanceIndex (d.xP, d.yP, "tauVel2", d.tauVel2, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs, levels=15);
fprintf("GtauAcc1 = %f\n", d.GtauAcc1);
fprintf("GtauVel1 = %f\n", d.GtauVel1);
fprintf("GtauAcc2 = %f\n", d.GtauAcc2);
fprintf("GtauVel2 = %f\n\n", d.GtauVel2);

%% Max torques
fprintf("Tray position = %f\n", d.trayPosition);

plotPerformanceIndex (d.xP, d.yP, "Tspec1", d.Tspec1, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
plotPerformanceIndex (d.xP, d.yP, "Tspec2", d.Tspec2, "theoreticalWorkspace", [a; b; w], onlyAboveX=onlyAboveX, workspaceBounds=d.rectangleCoordIDs);
fprintf("Tmax1 = %f\n", d.Tmax1);
fprintf("Tmax2 = %f\n\n", d.Tmax2);

fprintf("Trmsmax1 = %f\n", d.Trmsmax1);
fprintf("Trmsmax2 = %f\n\n", d.Trmsmax2);

%% Plot torque component contributions.
% figure(), contourf((cell2mat(percentageTa1)')), colorbar
% figure(), contourf((cell2mat(percentageTa2)')), colorbar

%%
WMids = getWMids(d.rectangleWMs);

figure();
for i = 1:5:length(d.tx) 
    hold on
    if rotatedWS
        pT1 = plot3(d.xPscaled(d.yPs{i}(1), d.xPs{i}), d.yPscaled(d.yPs{i}(1), d.xPs{i}), d.T1{i}, Color="#0072BD", LineWidth=2);
        pT2 = plot3(d.xPscaled(d.yPs{i}(1), d.xPs{i}), d.yPscaled(d.yPs{i}(1), d.xPs{i}), d.T2{i}, Color="#D95319", LineWidth=2);
    else 
        pT1 = plot3(d.xPscaled(d.yPs{i}, d.xPs{i}(1)), d.yPscaled(d.yPs{i}, d.xPs{i}(1)), d.T1{i}, Color="#0072BD", LineWidth=2);
        pT2 = plot3(d.xPscaled(d.yPs{i}, d.xPs{i}(1)), d.yPscaled(d.yPs{i}, d.xPs{i}(1)), d.T2{i}, Color="#D95319", LineWidth=2);
    end
    [T1control, T2control, txcontrol, qpcontrol] = dynamicsRunSimulation(d.robot, WMs{WMids(2, i)},  d.xStart(i), d.xStop(i), d.yStart(i), d.yStop(i), setpoint.vMax, setpoint.aMax, setpoint.jMax, driveAtElbows = driveAtElbows);
    pT1c = plot3(qpcontrol(1, :), qpcontrol(2, :), T1control, Color="#EDB120", LineWidth=2);
    pT2c = plot3(qpcontrol(1, :), qpcontrol(2, :), T2control, Color="#7E2F8E", LineWidth=2);
end
pLCI = surf(d.xPscaled, d.yPscaled, zeros(size(d.xP,1), size(d.xP,2)), d.LCIstitched, 'edgecolor', 'none', 'FaceAlpha', 0.6);
surf(d.xPscaled, d.yPscaled, -0.01*ones(size(d.xP,1), size(d.xP,2)), d.LCI(:, :, 1), 'edgecolor', 'none', 'FaceAlpha', 0.2);
plotRectangle(d.xPscaled, d.yPscaled, d.rectangleCoordIDs);
c = colorbar;
c.Label.String = "LCI [-]";
legend([pT1, pT2, pT1c, pT2c, pLCI], ["T1 - Tilde [Nm]", "T2 - Tilde [Nm]", "T1 - Full sim [Nm]", "T2 - Full sim [Nm]", "LCI [-]"]);
axis normal
xlabel("x [m]");
ylabel("y [m]");
zlabel("Torque [Nm]");
xlim([-6/2 * d.scalingD, 6/2 * d.scalingD]);
ylim([-6/2 * d.scalingD, 6/2 * d.scalingD]);
zlim([-90, 90]);
fontsize(16,"points"); 
daspect([1, 1, 200])
view(135, 45)
% view(90, 0)
grid on

%%
warning('on','all')
TileFigures
toc
