function plotTheoreticalWorkspace(a, b, w)
% plotDesignSpace - Plots the limits of the theoretical workspace for a
% given robot by tracing the loci of the type 1 singularities.
% 
% plotTheoreticalWorkspace(a, b, w)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
% 
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

hold on
fimplicit(@(x,y) (x+w).^2 + y.^2 - (a+b)^2, '-.k');
fimplicit(@(x,y) (x+w).^2 + y.^2 - (a-b)^2, '-.k');
fimplicit(@(x,y) (x-w).^2 + y.^2 - (a+b)^2, '-.k');
fimplicit(@(x,y) (x-w).^2 + y.^2 - (a-b)^2, '-.k');
hold off