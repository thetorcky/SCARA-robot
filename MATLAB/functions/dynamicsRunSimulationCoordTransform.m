function [T1, T2, tx, q] = dynamicsRunSimulationCoordTransform(robot, wm, xStart, xStop, yStart, yStop, vMax, aMax, jMax, options)
% dynamicsRunSimulationCoordTransform - Runs a dynamics simulation between 
% a start point and a stop point for the robot provided, taking into 
% account limits on the magnitude of the velocity, acceleration and jerk. 
% Returns the torque required to follow the inferred trajectory. 
% Calculations based on the coordinate transformed versions of the
% equations in dynamicsRunSimulation.
%
% [T1, T2, tx, q] = dynamicsRunSimulationTaskSpace(robot, wm, xStart, xStop, ...
%                yStart, yStop, vMax, aMax, jMax)
% INPUTS:   robot       = struct containing the various mechanical
%                         properties:
%           - rX            = length of link X [m]
%           - mX            = mass of link X [kg]
%           - rgX           = position of center of mass of link X [m]
%           - JX            = mass moment of inertia around the centroid 
%                             for link X [kgm2]
%           wm          = working mode branch indices for the WM used for
%                         this movwement.
%           xStart      = X-coordinate of the start point [m]
%           xStop       = X-coordinate of the stop point [m]
%           yStart      = Y-coordinate of the start point [m]
%           yStop       = Y-coordinate of the stop point [m]
%           vMax        = maximum velocity limit [m/s]
%           aMax        = maximum acceleration limit[m/s2]
%           jMax        = maximum jerk limit[m/s3]
%           options     = name-value pairs cointaining plotting preferences
%           - driveAtElbows         = drive position selection. 1 for
%                                     driving at elbows relative to the 
%                                     ground. 2 for driving at the elbows,
%                                     between the links. 0 for driving at
%                                     the shoulders.
% OUTPUTS:  
%           T1          = row vector containing the torque requirement for
%                         motor 1 [Nm]
%           T2          = row vector containing the torque requirement for
%                         motor 2 [Nm]
%           tx          = row vector containing sampled time instants [s]
%           q           = row of column vectors containing the coordinates
%                         of the end effector at each time instant [m]
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    robot struct 
    wm double
    xStart double
    xStop double
    yStart double
    yStop double
    vMax double
    aMax double
    jMax double
    options.driveAtElbows (1,1) int8 = 0
end

Ts = 0.0005;
A = robot.A;
B = robot.B;
W = robot.W;

if(xStart ~= xStop)
    disp("ERROR");
    return
end

[t, jd] = make3(yStop-yStart, vMax, aMax, jMax, Ts);
[~, tx, ~, a, ~, yP, ~] = profile3(t, jd(1), Ts, 0);
yP = transpose(yP) + yStart;
xP = xStart .* ones(1, length(tx));

q = [xP; yP];
dq = [zeros(2,1) diff(q,1,2)/Ts];
ddq = [zeros(2,1) diff(dq,1,2)/Ts];

[theta1, theta2] = IK(xP, yP, A, B, W, wm);
[beta1, beta2] = calculateFloatingAngles(A, B, W, xP, yP, theta1, theta2);

qang = [theta1; beta1; theta2; beta2];
dqang = [zeros(4,1) diffSafe(qang,1,2)/Ts];
ddqang = [zeros(4,1) diffSafe(dqang,1,2)/Ts];

if options.driveAtElbows == 1
    S = [0, 0;
         1, 0;
         0, 0; 
         0, 1];
elseif options.driveAtElbows == 2
    S = [-1, 0;
         1, 0;
         0, -1; 
         0, 1];
else
    S = [1, 0;
         0, 0;
         0, 1; 
         0, 0];
end

T1 = nan(1, length(tx));
T2 = T1;

for i = 1:length(tx)
    qi = q(:,i);
    dqi = dq(:,i);
    ddqi = ddq(:,i);
    qiang = qang(:,i);
    dqiang = dqang(:,i);
    ddqiang = ddqang(:,i);

    [M, C, ~, ~] = dynamicsAssembleMatrices(robot, qiang, dqiang);

    T = calculateJacobianExtended(A, B, W, qiang);
    TT = transpose(T);
    dT = calculateJacobianExtendedDerivative (A, B, W, qiang, dqiang);

    Me = TT*M*T;
    Ce = TT*(M*dT + C*T);
    Se = TT*S;

    lhs = Se;
    rhs = Me*ddqi + Ce*dqi;

    tau = lhs\rhs;
    T1(i) = tau(1);
    T2(i) = tau(2);
    
end

%%
% figure()
% yyaxis left
% plot(tx, T1, 'LineWidth',2);
% ylim([-42, 42]);
% hold on
% yyaxis right
% plot(tx, dqang(1, :), tx, dqang(2, :), 'LineWidth',2);
% plot(tx, ddqang(1, :), tx, ddqang(2, :), 'LineWidth',2);
% ylim([-200, 200])
% fontsize(16,"points")
% % plot(tx, a);
% legend(["T1 [Nm]", "$\dot{\theta_1} [rad/s]$", "$\dot{\beta_1} [rad/s]$", "$\ddot{\theta_1} [rad/s^2]$", "$\ddot{\beta_1} [rad/s^2]$"], 'Interpreter', 'latex');
% xlim([0, 0.120])
% title("Euler Lagrange")
% grid on


% %% Constraint error plotting
% h = [A*cos(q(1,:)) + B*cos(q(2,:)) - A*cos(q(3,:)) - B*cos(q(4,:)) - 2*robot.W;
%      A*sin(q(1,:)) + B*sin(q(2,:)) - A*sin(q(3,:)) - B*sin(q(4,:))];
% 
% figure()
% plot(tx, h(1,:), tx, h(1,:));
% title("Constraint equation error")
% legend(["X constraint", "Y constraint"])
