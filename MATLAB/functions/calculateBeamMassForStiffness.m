function m = calculateBeamMassForStiffness(C, L, E, rho)
% calculateBeamMassForStiffness - Calculates an estimate for the mass of a
% beam based on a required stiffness for it. Assumptions:
% - The beam is only loaded as a cantilever with a force on the free end:
%       Cy  = 3*E*Ix / L^3
% - The beam is a rectangular boxtube with cross section and thickness:
%       h = 2*w
%       t = 0.1 * w
% 
% m = calculateBeamMassForStiffness(C, L, E, rho)
% INPUTS:   C           = required stiffness [N/m]
%           L           = beam length [m]
%           E           = Young's modulus of the beam material [Pa]
%           rho         = density of the beam material [kg/m3]
% OUTPUTS:  m           = mass of the resulting beam
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

wo = nthroot(C*L^3/(4*E*(1-0.8^4)), 4);

wi = 0.8*wo;
ho = 2*wo;
hi = 0.8*ho;

m = rho * L * (wo*ho - wi*hi);
