function [M, C, WT, wbar, Chat] = dynamicsAssembleMatrices(robot, q, dq)
% dynamicsAssembleMatrices - Assembles the mass, coriollis and constriant
% matrices as part of the Euler Lagrnage equations of motion. 
% 
% [M, C, WT, wbar, Chat] = dynamicsAssembleMatrices(robot, q, dq)
% INPUTS:   robot       = struct containing the various mechanical
%                         properties:
%           - rX            = length of link X [m]
%           - mX            = mass of link X [kg]
%           - rgX           = position of center of mass of link X [m]
%           - JX            = mass moment of inertia around the centroid 
%                             for link X [kgm2]
%           q           = coordinate vector of the assembled system at a 
%                         given pose = [theta1; beta1; theta1; beta1]
%           dq          = velocity vector of the assembled system at a 
%                         given situation = [dtheta1; dbeta1; dtheta1; ...
%                         dbeta1]
% OUTPUTS:  
%           M           = 4x4 assembled mass matrix
%           C           = 4x4 assembled Coriolis matrix
%           WT          = 2x4 constraint matrix on velocity level
%           wbar        = 2x4 constraint matrix time derivative
%           Chat        = 4x1 cell array containing diagonal matrices
%                         Chati, used to express the terms in C
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

    A = robot.A;
    B = robot.B;
    W = robot.W;

    [M1, C1, Chati1, Chati2] = dynamicsMatricesArmi(robot, 1, q, dq);
    [M2, C2, Chati3, Chati4] = dynamicsMatricesArmi(robot, 2, q, dq);
    
    square0 = zeros(2, 2);

    M = [M1, square0; square0, M2];
    C = [C1, square0; square0, C2];

    WT = [-A*sin(q(1))     -B*sin(q(2))      A*sin(q(3))   B*sin(q(4));
           A*cos(q(1))      B*cos(q(2))     -A*cos(q(3))  -B*cos(q(4))];

    wbar = [-A*cos(q(1))*dq(1)^2 - B*cos(q(2))*dq(2)^2 + A*cos(q(3))*dq(3)^2 + B*cos(q(4))*dq(4)^2;
            -A*sin(q(1))*dq(1)^2 - B*sin(q(2))*dq(2)^2 + A*sin(q(3))*dq(3)^2 + B*sin(q(4))*dq(4)^2];

    Chat = {diag([0, Chati1, 0, 0]);
            diag([Chati2, 0, 0, 0]);
            diag([0, 0, 0, Chati3]);
            diag([0, 0, Chati4, 0])};

    % Constraint stabilisation
    % betastab = 1;
    % alphastab = 80;
    % wbar = wbar + 2*alphastab*betastab*WT*dq + (alphastab^2) * ...
    %     [A*cos(q(1)) + B*cos(q(2)) - A*cos(q(3)) - B*cos(q(4)) - 2*W;
    %      A*sin(q(1)) + B*sin(q(2)) - A*sin(q(3)) - B*sin(q(4))];

end

function [Mi, Ci, Chati1, Chati2] = dynamicsMatricesArmi(robot, i, q, dq)
    A = robot.A;
    B = robot.B;
    
    if i == 1
        mi1 = robot.m1;
        mi2 = robot.m2;
        mip = robot.m5 / 2;
        ri1 = robot.rg1;
        ri2 = robot.rg2;
        Ji1 = robot.J1;
        Ji2 = robot.J2;
        thetai = q(1);
        betai = q(2);
        dthetai = dq(1);
        dbetai = dq(2);
    else
        mi1 = robot.m3;
        mi2 = robot.m4;
        mip = robot.m5 / 2;
        ri1 = robot.rg3;
        ri2 = robot.rg4;
        Ji1 = robot.J3;
        Ji2 = robot.J4;
        thetai = q(3);
        betai = q(4);
        dthetai = dq(3);
        dbetai = dq(4);
    end

    Mi11 = (mi2 + mip) * A^2 + mi1*ri1^2 + Ji1;
    Mi12 = A * (mi2*ri2 + mip*B) * cos(thetai - betai);
    Mi21 = Mi12;
    Mi22 = mip * B^2 + mi2*ri2^2 + Ji2;

    Mi = [Mi11, Mi12; Mi21, Mi22];

    Chati1 = A * sin(thetai-betai) * (mi2*ri2 + mip*B);
    Chati2 = -A * sin(thetai-betai) * (mi2*ri2 + mip*B);

    Ci = [0, Chati1 * dbetai; Chati2 * dthetai, 0];

end
