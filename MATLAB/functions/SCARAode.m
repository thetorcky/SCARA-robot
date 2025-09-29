function xdot=SCARAode(t, x)

global robot tauData;
  
% Unpack state vector
q = x(1:4);       % Generalized coordinates
q_dot = x(5:8); % Generalized velocities

% Define parameters and system properties (mass, force, constraints)
[M, C, WT, wbar] = dynamicsAssembleMatrices(robot, q, q_dot);
W = transpose(WT);
invM = inv(M);
H = C*q_dot;
S = [1, 0;
     0, 0;
     0, 1; 
     0, 0];
tau1 = interp1(tauData(1,:), tauData(2,:), t);
tau2 = interp1(tauData(1,:), tauData(3,:), t);
Stau = S*[tau1; tau2];

xdot=[q_dot;
      invM*(Stau-H+W*inv(WT*invM*W)*(WT*invM*(H-Stau)-wbar))]; 

