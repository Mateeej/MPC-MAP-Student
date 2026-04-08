% function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
% %PREDICT_POSE Summary of this function goes here
% 
% new_pose = old_pose;
% 
% end

function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Applies motion model with noise to a single particle

dt   = read_only_vars.sampling_period;      % 0.1s
L    = read_only_vars.agent_drive.interwheel_dist; % 0.2m

vR = motion_vector(1);
vL = motion_vector(2);

% Linear and angular velocity from wheel speeds
v     = (vR + vL) / 2;
omega = (vR - vL) / L;

% Add noise to motion (probabilistic motion model)
SIGMA_V     = 0.05;   % noise on linear velocity
SIGMA_OMEGA = 0.05;   % noise on angular velocity
v     = v     + randn * SIGMA_V;
omega = omega + randn * SIGMA_OMEGA;

% Update pose
theta     = old_pose(3) + omega * dt;
new_pose  = [old_pose(1) + v * cos(theta) * dt, ...
             old_pose(2) + v * sin(theta) * dt, ...
             theta];
end

