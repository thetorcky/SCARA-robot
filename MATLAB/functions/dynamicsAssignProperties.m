function robot = dynamicsAssignProperties(A, B, W, varargin)
% dynamicsAssignProperties - Assign physical properties to a robot struct 
% based on data in the varargin argument. If no speciffic properties are
% communicated, then the default densities are used to calculate the 
% 
% robot = dynamicsAssignProperties(A, B, W, varargin)
% INPUTS:   A           = driven link length
%           B           = floating link length
%           W           = half of the base link length
%           varargin    = optional cell array containing the masses, CoM 
%                         positions and moments of inertia 
% OUTPUTS:  robot       = struct containing the various mechanical
%                         properties:
%           - rX            = length of link X [m]
%           - mX            = mass of link X [kg]
%           - rgX           = position of center of mass of link X [m]
%           - JX            = mass moment of inertia around the centroid 
%                             for link X [kgm2]
%           where X = 
%           - 1 for the driven link of arm 1
%           - 2 for the floating link of arm 1
%           - 3 for the driven link of arm 2
%           - 4 for the floating link of arm 2
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

if isempty(varargin)
    robot.A=A;                                  %[m] length of link1 & 3
    robot.B=B;                                  %[m] length of link2 & 4
    robot.W=W;
    
    r1 = robot.A;
    r2 = robot.B;
    r3 = robot.A;
    r4 = robot.B;
    
    robot.m5 = 0.300;                           %[kg] mass of picker
    specStiffness = 1e8;    % Desired system stiffness * 100 [N/m]
    E = 68e9;               % Young's modulus [Pa] - 6061 Alu
    rho = 2700;             % Density [kg/m3] - 6061 Alu
    robot.m1 = calculateBeamMassForStiffness(100*specStiffness, r1, E, rho) + 1;                  %[kg] mass of link1
    robot.m2 = calculateBeamMassForStiffness(specStiffness, r2, E, rho) + 0.3;                  %[kg] mass of link2
    robot.m3 = calculateBeamMassForStiffness(100*specStiffness/2, r3, E, rho) + 1;                  %[kg] mass of link3
    robot.m4 = calculateBeamMassForStiffness(specStiffness/2, r4, E, rho) + 0.3;                  %[kg] mass of link4
    % massDensity = 0.185/0.288;
    % robot.m5 = 0.200;                           %[kg] mass of picker
    % robot.m1 = massDensity*r1;                  %[kg] mass of link1
    % robot.m2 = massDensity*r2;                  %[kg] mass of link2
    % robot.m3 = massDensity*r3;                  %[kg] mass of link3
    % robot.m4 = massDensity*r4;                  %[kg] mass of link4
    
    robot.rg1 = r1/2;                           %[m] length to CoG of link1
    robot.rg2 = r2/2;                           %[m] length to CoG of link2
    robot.rg3 = r3/2;                           %[m] length to CoG of link3
    robot.rg4 = r4/2;                           %[m] length to CoG of link4
    
    robot.J1 = robot.m1*r1*r1/12;               %[kgm2] Inertia of link1
    robot.J2 = robot.m2*r2*r2/12;               %[kgm2] Inertia of link2
    robot.J3 = robot.m3*r3*r3/12;               %[kgm2] Inertia of link3
    robot.J4 = robot.m4*r4*r4/12;               %[kgm2] Inertia of link4
    
else
    masses = varargin{1};
    rgs = varargin{2};
    inertias = varargin{3};

    robot.A=A;                                  %[m] length of link 1 & 3
    robot.B=B;                                  %[m] length of link 2 & 4
    robot.W=W;
    
    if length(masses) == 3
        robot.m5 = masses(3);                       %[kg] mass of picker
        robot.m1 = masses(1);                       %[kg] mass of link1
        robot.m2 = masses(2);                       %[kg] mass of link2
        robot.m3 = masses(1);                       %[kg] mass of link3
        robot.m4 = masses(2);                       %[kg] mass of link4
        
        robot.rg1 = rgs(1);                         %[m] length to CoG of link1
        robot.rg2 = rgs(2);                         %[m] length to CoG of link2
        robot.rg3 = rgs(1);                         %[m] length to CoG of link3
        robot.rg4 = rgs(2);                         %[m] length to CoG of link4
    
        robot.J1 = inertias(1);                     %[kgm2] Inertia of link1
        robot.J2 = inertias(2);                     %[kgm2] Inertia of link2
        robot.J3 = inertias(1);                     %[kgm2] Inertia of link3
        robot.J4 = inertias(2);                     %[kgm2] Inertia of link4

    elseif length(masses) == 5
        robot.m5 = masses(5);                       %[kg] mass of picker
        robot.m1 = masses(1);                       %[kg] mass of link1
        robot.m2 = masses(2);                       %[kg] mass of link2
        robot.m3 = masses(3);                       %[kg] mass of link3
        robot.m4 = masses(4);                       %[kg] mass of link4
        
        robot.rg1 = rgs(1);                         %[m] length to CoG of link1
        robot.rg2 = rgs(2);                         %[m] length to CoG of link2
        robot.rg3 = rgs(3);                         %[m] length to CoG of link3
        robot.rg4 = rgs(4);                         %[m] length to CoG of link4

        robot.J1 = inertias(1);                     %[kgm2] Inertia of link1
        robot.J2 = inertias(2);                     %[kgm2] Inertia of link2
        robot.J3 = inertias(3);                     %[kgm2] Inertia of link3
        robot.J4 = inertias(4);                     %[kgm2] Inertia of link4
    end

end
