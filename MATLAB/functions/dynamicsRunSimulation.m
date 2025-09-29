function [T1, T2, tx, qp] = dynamicsRunSimulation(robot, wm, xStart, xStop, yStart, yStop, vMax, aMax, jMax, options)
% dynamicsRunSimulation - Runs a dynamics simulation between a start point
% and a stop point for the robot provided, taking into account limits on
% the magnitude of the velocity, acceleration and jerk. Returns the torque
% required to follow the inferred trajectory.
% 
% [T1, T2, tx, qp] = dynamicsRunSimulation(robot, wm, xStart, xStop, yStart, ...
%                yStop, vMax, aMax, jMax)
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

if yStop == yStart
    moveAlong = 2;
    activeStart = xStart;
    activeStop = xStop;
    passiveStart = yStart;
elseif xStop == xStart
    moveAlong = 1;
    activeStart = yStart;
    activeStop = yStop;
    passiveStart = xStart;
else 
    error("Motion is not along one axis");
end
    
[t, jd] = make3(activeStop - activeStart, vMax, aMax, jMax, Ts);
[~, tx, ~, ~, ~, activePP, ~] = profile3(t, jd(1), Ts, 0);
activePP = sign(activeStop - activeStart) * transpose(activePP) + activeStart;
passivePP = passiveStart .* ones(1, length(tx));

if moveAlong == 2
    qp = [activePP; passivePP];
else
    qp = [passivePP; activePP];
end

[theta1, theta2] = IK(qp(1, :), qp(2, :), A, B, W, wm);
[beta1, beta2] = calculateFloatingAngles(A, B, W, qp(1, :), qp(2, :), theta1, theta2);
q = [theta1; beta1; theta2; beta2];

dq = [zeros(4,1) diffSafe(q,1,2)/Ts];
ddq = [zeros(4,1) diffSafe(dq,1,2)/Ts];

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

    [M, C, WT, wbar] = dynamicsAssembleMatrices(robot, qi, dqi);
    W = transpose(WT);

    lhs = (eye(4) - W*inv(WT*inv(M)*W)*WT*inv(M))*S;
    rhs = M*ddqi + C*dqi - W*inv(WT*inv(M)*W) * (WT*inv(M)*C*dqi - wbar);

    tau = lhs\rhs;
    T1(i) = tau(1);
    T2(i) = tau(2);
    
end

% theta1 = theta1 - theta1(1);
% theta2 = theta2 - theta2(1);
% 
% theta1deg = rad2deg(theta1);
% theta2deg = rad2deg(theta2);
% 
% theta1deg = [transpose(tx) transpose(theta1deg)];
% theta2deg = [transpose(tx) transpose(theta2deg)];
% 
% writematrix(theta1deg, "theta1deg" + "_Ts=" + num2str(Ts) + "_x=" + num2str(xStart) + ".csv");
% writematrix(theta2deg, "theta2deg" + "_Ts=" + num2str(Ts) + "_x=" + num2str(xStart) + ".csv");

% beta1 = beta1 - beta1(1);
% beta2 = beta2 - beta2(1);
% 
% beta1deg = rad2deg(beta1);
% beta2deg = rad2deg(beta2);
% 
% beta1deg = [transpose(tx) transpose(beta1deg)];
% beta2deg = [transpose(tx) transpose(beta2deg)];
% 
% writematrix(beta1deg, "beta1degtest" + "_Ts=" + num2str(Ts) + "_x=" + num2str(xStart) + ".csv");
% writematrix(beta2deg, "beta2degtest" + "_Ts=" + num2str(Ts) + "_x=" + num2str(xStart) + ".csv");

%%
% figure()
% yyaxis left
% plot(tx, T1, 'LineWidth',2);
% ylim([-6, 6]);
% hold on
% yyaxis right
% plot(tx, dq(1, :), tx, dq(2, :), 'LineWidth',2);
% plot(tx, ddq(1, :), tx, ddq(2, :), 'LineWidth',2);
% ylim([-600, 600])
% fontsize(16,"points")
% 
% % plot(tx, a);
% legend(["T1 [Nm]", "$\dot{\theta_1} [rad/s]$", "$\dot{\beta_1} [rad/s]$", "$\ddot{\theta_1} [rad/s^2]$", "$\ddot{\beta_1} [rad/s^2]$"], 'Interpreter', 'latex');
% xlim([0, 0.120])
% title("Euler Lagrange")
% grid on

%% Constraint error plotting
% h = [A*cos(q(1,:)) + B*cos(q(2,:)) - A*cos(q(3,:)) - B*cos(q(4,:)) - 2*robot.W;
%      A*sin(q(1,:)) + B*sin(q(2,:)) - A*sin(q(3,:)) - B*sin(q(4,:))];
% 
% figure()
% plot(tx, h(1,:), tx, h(1,:));
% title("Constraint equation error")
% legend(["X constraint", "Y constraint"])


