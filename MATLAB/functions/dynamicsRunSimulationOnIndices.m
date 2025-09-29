function [T1, T2, tx, xPs, yPs, percentageTa1, percentageTa2, spacialT1, spacialT2] = dynamicsRunSimulationOnIndices(xP, yP, Metilda, Hetilda1, Hetilda2, xStart, xStop, yStart, yStop, vMax, aMax, jMax, Ts)
% dynamicsRunSimulationOnIndices - Runs a dynamics simulation between a 
% start point and a stop point for the robot provided, taking into account limits on
% the magnitude of the velocity, acceleration and jerk. Returns the torque
% required to follow the inferred trajectory. Uses the pre-calculated
% pose-dependant dynamics matrices.
% 
% [T1, T2, tx, xPs, yPs] = dynamicsRunSimulationOnIndices(xP, yP, ...
%                           Metilda, Hetilda1, Hetilda2, xStart, xStop, ...
%                           yStart, yStop, vMax, aMax, jMax)
% INPUTS:   xP          = nSteps x nSteps matrix of X coordinates
%           yP          = nSteps x nSteps matrix of Y coordinates
%           Metilda     = nSteps x nSteps cell array of the Metilda matrix
%                         for each pose
%           Hetilda1    = nSteps x nSteps cell array of the Hetilda1 matrix
%                         for each pose, the first row of Hetilda
%           Hetilda2    = nSteps x nSteps cell array of the Hetilda2 matrix
%                         for each pose, the second row of Hetilda
%           xStart      = X-coordinate of the start point [m]
%           xStop       = X-coordinate of the stop point [m]
%           yStart      = Y-coordinate of the start point [m]
%           yStop       = Y-coordinate of the stop point [m]
%           vMax        = maximum velocity limit [m/s]
%           aMax        = maximum acceleration limit [m/s2]
%           jMax        = maximum jerk limit [m/s3]
%           Ts          = time discretization step [s] (default = 0.0005 s)
% OUTPUTS:  T1          = row vector containing the torque requirement for
%                         motor 1 [Nm]
%           T2          = row vector containing the torque requirement for
%                         motor 2 [Nm]
%           tx          = row vector containing sampled time instants [s]
%           xPs         = row vector containing the indices of the 
%                         discretized workspace points used at each step of
%                         the simulation
%           yPs         = row vector containing the indices of the 
%                         discretized workspace points used at each step of
%                         the simulation
%           percentageTa1 = row vector containing the percentage of 
%                           contribution of the acceleration term to the
%                           torque of motor 1 at each time step.
%           percentageTa2 = row vector containing the percentage of 
%                           contribution of the acceleration term to the
%                           torque of motor 2 at each time step.
%           spacialT1   = row vector with the maximum torque encountered on 
%                         motor 1 at all poses (along the y axis) in the WS
%           spacialT2   = row vector with the maximum torque encountered on 
%                         motor 2 at all poses (along the y axis) in the WS
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

if nargin < 13
    Ts = 0.0005;
end

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
dqp = [zeros(2,1) diff(qp,1,2)/Ts];
ddqp = [zeros(2,1) diff(dqp,1,2)/Ts];

% wm = [-1, -1];
% 
% % vMaxMotor = 13;
% % aMaxMotor = 500;
% % jMaxMotor = 20000;
% vMaxMotor = 13;
% aMaxMotor = 350;
% jMaxMotor = 17000;
% 
% [theta1start, theta2start] = IK(xStart, yStart, robot.A, robot.B, robot.W, wm);
% [theta1stop, theta2stop] = IK(xStop, yStop, robot.A, robot.B, robot.W, wm);
% 
% [theta1start, theta1stop] = patchAngularDiscontinuity(theta1start, theta1stop);
% [theta2start, theta2stop] = patchAngularDiscontinuity(theta2start, theta2stop);
% 
% [t, jd] = make3(theta1stop - theta1start, vMaxMotor, aMaxMotor, jMaxMotor, Ts);
% [~, tx1, ~, ~, ~, theta1, ~] = profile3(t, jd(1), Ts, 1);
% theta1 = sign(theta1stop - theta1start) * transpose(theta1) + theta1start;
% [t, jd] = make3(theta2start - theta2stop, vMaxMotor, aMaxMotor, jMaxMotor, Ts);
% [~, tx2, ~, ~, ~, theta2, ~] = profile3(t, jd(1), Ts, 1);
% theta2 = sign(theta2stop - theta2start) * transpose(theta2) + theta2start;
% 
% txLen = max(length(tx1), length(tx2));
% tx = 0:Ts:(txLen - 1) * Ts;
% theta1 = [theta1, theta1(end) * ones(1, max(0, length(theta2)-length(theta1)))];
% theta2 = [theta2, theta2(end) * ones(1, max(0, length(theta1)-length(theta2)))];
% 
% [xP, yP, ~, ~, ~, ~] = FK(theta1, theta2, robot.A, robot.B, robot.W);
% qp = [xP; yP];
% dqp = [zeros(2,1) diff(qp,1,2)/Ts];
% ddqp = [zeros(2,1) diff(dqp,1,2)/Ts];

T1 = nan(1, length(tx));
T2 = T1;
xPs = T1;
yPs = T1;
percentageTa1 = T1;
percentageTa2 = T2;
spacialT1 = nan(1, size (xP, moveAlong));
spacialT2 = spacialT1;

for i = 1:length(tx)
    qpi = qp(:,i);
    dqpi = dqp(:,i);
    ddqpi = ddqp(:,i);

    [~, xPindex] = min(abs(xP(1,:)-qpi(1)));
    [~, yPindex] = min(abs(yP(:,1)-qpi(2)));
    xPs(i) = xPindex;
    yPs(i) = yPindex;

    if(~isempty(Metilda{yPindex, xPindex}))
        Ta = Metilda{yPindex, xPindex} * ddqpi;
        Tv1 = dqpi' * Hetilda1{yPindex, xPindex} * dqpi;
        Tv2 = dqpi' * Hetilda2{yPindex, xPindex} * dqpi;
    
        T1(i) = Ta(1) + Tv1;
        T2(i) = Ta(2) + Tv2;

        if moveAlong == 2
            spacialT1(xPs(i)) = max(spacialT1(xPs(i)), T1(i));
            spacialT2(xPs(i)) = max(spacialT2(xPs(i)), T2(i));
        else
            spacialT1(yPs(i)) = max(spacialT1(yPs(i)), T1(i));
            spacialT2(yPs(i)) = max(spacialT2(yPs(i)), T2(i));
        end
        
        % percentageTa1(i) = abs(Ta(1)) / abs(T1(i));
        % percentageTa2(i) = abs(Ta(2)) / abs(T2(i));
        percentageTa1(i) = abs(Ta(1)) / (abs(T1(i)) + abs(Tv1));
        percentageTa2(i) = abs(Ta(2)) / (abs(T2(i)) + abs(Tv2));
        % percentageTa1(i) = abs(Ta(1)) / abs(Tv1);
        % percentageTa2(i) = abs(Ta(2)) / abs(Tv2);
        % percentageTa1(i) = Ta(1)^2 / (T1(i)^2 + Tv1^2);
        % percentageTa2(i) = Ta(2)^2 / (T2(i)^2 + Tv2^2);
    end
end
