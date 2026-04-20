function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Pure Pursuit path following using MoCap pose.
 
% ---- Parameters ----
K_angular  = 4.0;   % how aggressively to turn toward target
V_base     = 1.0;   % base forward speed (max 1.0)
INTERWHEEL = read_only_vars.agent_drive.interwheel_dist; % 0.2 m
 
% ---- Get current pose from MoCap ----
%pose = read_only_vars.mocap_pose; % [x, y, theta]
pose = public_vars.estimated_pose;
 
if isempty(pose) || isempty(public_vars.path)
    public_vars.motion_vector = [0, 0];
    return;
end
 
% ---- Get current target waypoint ----
[target, public_vars] = get_target(pose, public_vars);
 
% ---- Compute angle to target ----
dx = target(1) - pose(1);
dy = target(2) - pose(2);
angle_to_target = atan2(dy, dx);
 
% ---- Heading error ----
heading_error = angle_to_target - pose(3);
heading_error = atan2(sin(heading_error), cos(heading_error)); % wrap
 
% ---- Differential drive: convert to wheel speeds ----
% omega = angular velocity, v = linear velocity
omega = K_angular * heading_error;
v     = V_base * cos(heading_error); % slow down when turning sharply
v     = max(0, v);                   % don't go backwards
 
% Convert (v, omega) to (vR, vL)
vR = v + (omega * INTERWHEEL / 2);
vL = v - (omega * INTERWHEEL / 2);
 
% Clamp to [-1, 1]
max_vel = read_only_vars.agent_drive.max_vel;
vR = max(-max_vel, min(max_vel, vR));
vL = max(-max_vel, min(max_vel, vL));
 
public_vars.motion_vector = [vR, vL];
%public_vars.motion_vector = [0, 0];
 
end