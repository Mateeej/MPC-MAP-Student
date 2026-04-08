function [target, public_vars] = get_target(pose, public_vars)
%GET_TARGET Returns the current waypoint target using Pure Pursuit logic.
% Advances to the next waypoint when robot is close enough.
 
WAYPOINT_RADIUS = 0.5; % metres — how close to count as "reached"
 
path = public_vars.path;
idx  = public_vars.target_idx;
 
% Clamp index to valid range
if idx > size(path, 1)
    idx = size(path, 1);
end
 
target = path(idx, :);
 
% Check if close enough to advance to next waypoint
dist = sqrt((pose(1) - target(1))^2 + (pose(2) - target(2))^2);
if dist < WAYPOINT_RADIUS && idx < size(path, 1)
    idx = idx + 1;
    target = path(idx, :);
    fprintf('Waypoint %d reached, moving to %d\n', idx-1, idx);
end
 
public_vars.target_idx = idx;
 
end

