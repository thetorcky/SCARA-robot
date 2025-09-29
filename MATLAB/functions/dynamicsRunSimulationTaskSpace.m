function [T1, T2, tx, qp] = dynamicsRunSimulationTaskSpace(robot, wm, xStart, xStop, yStart, yStop, vMax, aMax, jMax, options)
% dynamicsRunSimulationTaskSpace - Runs a dynamics simulation between a 
% start point and a stop point for the robot provided, taking into account limits on
% the magnitude of the velocity, acceleration and jerk. Returns the torque
% required to follow the inferred trajectory. (Mostly) using the
% independent coordinates (x,y).
% 
% [T1, T2, tx, qp] = dynamicsRunSimulationTaskSpace(robot, wm, xStart, xStop, ...
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
%           qp          = row of column vectors containing the coordinates
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

qp = [xP; yP];
dqp = [zeros(2,1) diffSafe(qp,1,2)/Ts];
ddqp = [zeros(2,1) diffSafe(dqp,1,2)/Ts];

[theta1, theta2] = IK(xP, yP, A, B, W, wm);
[beta1, beta2] = calculateFloatingAngles(A, B, W, xP, yP, theta1, theta2);

q = [theta1; beta1; theta2; beta2];
dq = [zeros(4,1) diffSafe(q,1,2)/Ts];
ddq = [zeros(4,1) diffSafe(dq,1,2)/Ts];

T1 = nan(1, length(tx));
T2 = T1;

for i = 1:length(tx)
    qpi = qp(:,i);
    dqpi = dqp(:,i);
    ddqpi = ddqp(:,i);
    qi = q(:,i);
    dqi = dq(:,i);
    % ddqi = ddq(:,i);

    [M, ~, ~, ~, Chat] = dynamicsAssembleMatrices(robot, qi, dqi);

    T = calculateJacobianExtended(A, B, W, qi);
    TT = transpose(T);

    delT = calculateDelT(A, B, W, qi);

    if options.driveAtElbows == 1
        JJ = [T(2,:); T(4,:)];
    elseif options.driveAtElbows == 2
        JJ = [T(2,:); T(4,:)] - [T(1,:); T(3,:)];
    else
        JJ = [T(1,:); T(3,:)];
    end
    
    Ce1 = inv(transpose(JJ)) * TT * M;
    Ce2 = inv(transpose(JJ)) * TT;
    Metilda = inv(transpose(JJ)) * TT * M * T;
    Hetilda1 = calculateHetildaj(1, T, TT, delT, Ce1, Ce2, Chat);
    Hetilda2 = calculateHetildaj(2, T, TT, delT, Ce1, Ce2, Chat);

    tau = Metilda * ddqpi;
    tau(1) = tau(1) + dqpi' * Hetilda1 * dqpi;
    tau(2) = tau(2) + dqpi' * Hetilda2 * dqpi;

    T1(i) = tau(1);
    T2(i) = tau(2);

end

%%
% figure()
% yyaxis left
% plot(tx, T1, 'LineWidth',2);
% ylim([-42, 42]);
% yyaxis right
% hold on
% % plot(tx, dq(1, :), tx, dq(2, :), 'LineWidth',2);
% % plot(tx, ddq(1, :), tx, ddq(2, :), 'LineWidth',2);
% plot(tx, q(1, :), tx, q(2, :), 'LineWidth',2);
% % ylim([-200, 200])
% fontsize(16,"points")
% % plot(tx, a);
% legend(["T1 [Nm]", "$\dot{\theta_1} [rad/s]$", "$\dot{\beta_1} [rad/s]$", "$\ddot{\theta_1} [rad/s^2]$", "$\ddot{\beta_1} [rad/s^2]$"], 'Interpreter', 'latex');
% xlim([0, 0.120])
% title("Euler Lagrange")
% grid on


% %% Constraint error plotting
% h = [A*cos(qp(1,:)) + B*cos(qp(2,:)) - A*cos(qp(3,:)) - B*cos(qp(4,:)) - 2*robot.W;
%      A*sin(qp(1,:)) + B*sin(qp(2,:)) - A*sin(qp(3,:)) - B*sin(qp(4,:))];
% 
% figure()
% plot(tx, h(1,:), tx, h(1,:));
% title("Constraint equation error")
% legend(["X constraint", "Y constraint"])
