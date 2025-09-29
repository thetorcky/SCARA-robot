function id = plotTrapezoid()
% plotTrapezoid - Plots data over the nondimensional design space
% trapezoid for symmetric 5-bar SCARA robots.
%  
% plotTrapezoid() - Creates an empty trapezoid plot of the nondimensional
% design space, including the singularity region boundaries and labels.
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

% hFigure = figure;
% set(hFigure, 'MenuBar', 'none');
% set(hFigure, 'ToolBar', 'none');

figure()
hold on

plot3([3 0 0 1.5; 0 0 1.5 3], [0 3 1.5 0; 3 1.5 0 0], [0 0 1.5 1.5; 0 1.5 1.5 0], '-', 'MarkerSize', 10, 'LineWidth',2, 'color', 'k', 'LineJoin', 'round', 'DisplayName', 'trapezoidBorder');
id = plot3([3 0 1.5 1.5 1.5; 0 1.5 1.5 0 0.75]+0.05*1.5, [0 3 1.5 1.5 1.5; 1.5 0 0 1.5 0.75]+0.05*1.5, [0 0 0 0 0; 1.5 1.5 1.5 1.5 1.5]+0.07*1.5, '--', 'LineWidth',1, 'color', 'k', 'LineJoin', 'round', 'DisplayName','trapezoidRegions');
plot3([3 0 1.5 1.5 1.5; 0 1.5 1.5 0 0.75]-0.05*1.5, [0 3 1.5 1.5 1.5; 1.5 0 0 1.5 0.75]-0.05*1.5, [0 0 0 0 0; 1.5 1.5 1.5 1.5 1.5]-0.07*1.5, '--', 'LineWidth',1, 'color', 'k', 'LineJoin', 'round', 'DisplayName','trapezoidRegions2');
view(135, 45)

xText = [1.1 1.35 2 2 1.4];
yText = [0.55 0.6 0.2 0.7 1];
zText = [1.35 1.05 0.8 0.3 0.6];
textLabels = {"Ia", "IIa", "IIIa", "IVa", "Va", "Ib", "IIb", "IIIb", "IVb", "Vb"};
text([xText yText+0.15], [yText xText-0.15], [zText zText]+0.02, textLabels, 'Color', 'blue', 'FontSize', 14);

hold off

xlabel("a (normalized) [-]");
ylabel("b (normalized) [-]");
zlabel("w (normalized) [-]");
axis equal;
grid on;
