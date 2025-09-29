function dJJ = calculateJacobianExtendedDerivative (a, b, w, q, dq)
% calculateJacobianExtendedDerivative - Calculates the time derivative of
% the extended Jacobian at a pose, for a specified velocity vector.
% 
% dJJ = calculateJacobianExtendedDerivative (a, b, w, q, dq)
% INPUTS:   a           = driven link length (nondimensional)
%           b           = floating link length (nondimensional)
%           w           = half of the base link length (nondimensional)
%           q           = coordinate vector of the assembled system at a 
%                         given pose = [theta1; beta1; theta1; beta1]
%           dq          = velocity vector of the assembled system at a 
%                         given situation = [dtheta1; dbeta1; dtheta1; ...
%                         dbeta1]
% OUTPUTS:  JJext       = 4 x 2 x nStepsY x nStepsX array containging the
%                         4 x 2 Jacobian matrices as pages
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

theta1 = q(1);
beta1 = q(2);
theta2 = q(3);
beta2 = q(4);
dtheta1 = dq(1);
dbeta1 = dq(2);
dtheta2 = dq(3);
dbeta2 = dq(4);

dJJ =   [  (2*dbeta1*cos(theta1))/(a*(cos(2*beta1 - 2*theta1) - 1)) + (dtheta1*cos(beta1)*cos(beta1 - theta1))/(a*sin(beta1 - theta1)^2),   (2*dbeta1*sin(theta1))/(a*(cos(2*beta1 - 2*theta1) - 1)) + (dtheta1*sin(beta1)*cos(beta1 - theta1))/(a*sin(beta1 - theta1)^2);
           (2*dtheta1*cos(beta1))/(b*(cos(2*beta1 - 2*theta1) - 1)) + (dbeta1*cos(theta1)*cos(beta1 - theta1))/(b*sin(beta1 - theta1)^2),   (2*dtheta1*sin(beta1))/(b*(cos(2*beta1 - 2*theta1) - 1)) + (dbeta1*sin(theta1)*cos(beta1 - theta1))/(b*sin(beta1 - theta1)^2);
           (2*dbeta2*cos(theta2))/(a*(cos(2*beta2 - 2*theta2) - 1)) + (dtheta2*cos(beta2)*cos(beta2 - theta2))/(a*sin(beta2 - theta2)^2),   (2*dbeta2*sin(theta2))/(a*(cos(2*beta2 - 2*theta2) - 1)) + (dtheta2*sin(beta2)*cos(beta2 - theta2))/(a*sin(beta2 - theta2)^2);
           (2*dtheta2*cos(beta2))/(b*(cos(2*beta2 - 2*theta2) - 1)) + (dbeta2*cos(theta2)*cos(beta2 - theta2))/(b*sin(beta2 - theta2)^2),   (2*dtheta2*sin(beta2))/(b*(cos(2*beta2 - 2*theta2) - 1)) + (dbeta2*sin(theta2)*cos(beta2 - theta2))/(b*sin(beta2 - theta2)^2)];

%% Derivation:
% syms theta1 beta1 theta2 beta2 a b
% syms dx dy
% syms dtheta1 dbeta1 dtheta2 dbeta2
% 
% JJext(1, 1) = cos(beta1) / (a * sin(beta1 - theta1));
% JJext(1, 2) = sin(beta1) / (a * sin(beta1 - theta1));
% JJext(2, 1) = -cos(theta1) / (b * sin(beta1 - theta1));
% JJext(2, 2) = -sin(theta1) / (b * sin(beta1 - theta1));
% JJext(3, 1) = cos(beta2) / (a * sin(beta2 - theta2));
% JJext(3, 2) = sin(beta2) / (a * sin(beta2 - theta2));
% JJext(4, 1) = -cos(theta2) / (b * sin(beta2 - theta2));
% JJext(4, 2) = -sin(theta2) / (b * sin(beta2 - theta2));
% 
% JJext
% 
% q = [theta1; beta1; theta2; beta2];
% dq = [dx; dy];
% 
% %JJext_qdot = JJext * dq
% JJext_qdot = [dtheta1; dbeta1; dtheta2; dbeta2];
% 
% JJdel1 = simplify(diff(JJext, theta1))
% JJdel2 = simplify(diff(JJext, beta1))
% JJdel3 = simplify(diff(JJext, theta2))
% JJdel4 = simplify(diff(JJext, beta2))
% 
% for i = 1:4
%     for j = 1:2
%         JJdel(i,j) = JJdel1(i,j) .* JJext_qdot(1) + JJdel2(i,j) .* JJext_qdot(2) + JJdel3(i,j) .* JJext_qdot(3) + JJdel4(i,j) .* JJext_qdot(4);
%     end
% end
% 
% JJdel = simplify (JJdel)