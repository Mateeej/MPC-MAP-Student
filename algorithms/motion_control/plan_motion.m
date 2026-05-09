function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Pure Pursuit path following with LiDAR reactive avoidance

% ---- Parameters ----
K_angular  = 4.0;
V_base     = 0.7;
INTERWHEEL = read_only_vars.agent_drive.interwheel_dist;

% ---- Init spin pre indoor lokalizáciu ----
if public_vars.pf_enabled && ~public_vars.init_spin_done
    if read_only_vars.counter < 60
        public_vars.motion_vector = [0.5, -0.5];
        return;
    else
        public_vars.init_spin_done = 1;
    end
end

% ---- Get current pose ----
pose = public_vars.estimated_pose;

if isempty(pose) || any(isnan(pose)) || isempty(public_vars.path)
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
heading_error = atan2(sin(heading_error), cos(heading_error));

% ---- LiDAR reactive avoidance ----
lidar = read_only_vars.lidar_distances;

% Predné 3 senzory: kanál 1 (0°), 2 (45°), 8 (315°)
front = lidar(1);
front_left  = lidar(2);  % 45°
front_right = lidar(8);  % 315°

AVOID_DIST = 0.6;  % metre

if isfinite(front) && front < AVOID_DIST
    % Prekážka vpredu - zatočí na stranu kde je viac miesta
    if front_left > front_right
        heading_error = heading_error - 0.5;  % zatočí vľavo
    else
        heading_error = heading_error + 0.5;  % zatočí vpravo
    end
elseif isfinite(front_left) && front_left < AVOID_DIST
    heading_error = heading_error - 0.3;  % mierne vpravo
elseif isfinite(front_right) && front_right < AVOID_DIST
    heading_error = heading_error + 0.3;  % mierne vľavo
end

heading_error = atan2(sin(heading_error), cos(heading_error));

% ---- Differential drive ----
omega = K_angular * heading_error;
v     = V_base * cos(heading_error);
v     = max(0, v);

vR = v + (omega * INTERWHEEL / 2);
vL = v - (omega * INTERWHEEL / 2);

max_vel = read_only_vars.agent_drive.max_vel;
vR = max(-max_vel, min(max_vel, vR));
vL = max(-max_vel, min(max_vel, vL));

public_vars.motion_vector = [vR, vL];

end