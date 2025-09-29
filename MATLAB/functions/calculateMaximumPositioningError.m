function MPE = calculateMaximumPositioningError (JJ, dtheta)
% calculateMaximumPositioningError - Calculates the maximum positioning
% error at the end effector for all poses in the workspace
%  
% MPE = calculateMaximumPositioningError (JJ, dtheta)
% INPUTS:   JJ           = 4-D array containing the Jacobian of the robot
%           dtheta       = maximum encoder error
%
% OUTPUTS:  MPE          = matrix containing the maximum positioning error
%                          at each pose
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

JJinv = pageinv(JJ);
dthetaVec = [dtheta, dtheta; -dtheta, dtheta];

Jv = pagemtimes(JJinv, dthetaVec);

allPE = vecnorm(Jv,2,1);
maxNorm = max(allPE,[],2); 
MPE(:,:) = maxNorm(1,1,:,:);

%% Debugging
% i = 150;
% j = 230;
% 
% figure()
% plot(dthetaVec(1,:), dthetaVec(2,:))
% xlabel("x encoder error [rad]")
% ylabel("y encoder error [rad]")
% axis equal
% grid on
% 
% thisJJinv = JJinv(:,:,i,j);
% theta = linspace(0, 2*pi, 1000); % Fine sampling of angles
% V = 0.001*sqrt(2)*[cos(theta); sin(theta)]; % Unit circle vectors
% elipsoid = thisJJinv * V;
% 
% figure()
% plot(elipsoid(1,:), elipsoid(2,:))
% xlabel("x encoder error [rad]")
% ylabel("y encoder error [rad]")
% axis equal
% grid on
% 
% % figure()
% hold on
% plot(Jv(1,:,i,j), Jv(2,:,i,j))
% xlabel("x end effector error [length]")
% ylabel("y end effector error [length]")
% axis equal
% grid on
% 
% figure()
% plot(allPE(:,:,i,j))
% xlabel("Encoder error direction")
% ylabel("End effector error magnitude [length]")
% grid on
% 
% disp(MPE(i,j))