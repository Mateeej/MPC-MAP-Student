function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

public_vars.motion_vector = [0, 0];

%t = read_only_vars.counter;
t = 6000;

if t < 160
    % Turn right to face ~0 degrees (east)
    public_vars.motion_vector = [0.49, 0.5];

elseif t < 190
    % Drive straight toward the goal
    public_vars.motion_vector = [0.5, 0.2];

elseif t < 350
    % Drive straight toward the goal
    public_vars.motion_vector = [0.49, 0.5];

elseif t < 380
    % Drive straight toward the goal
    public_vars.motion_vector = [0.2, 0.5];

elseif t < 450
    % Drive straight toward the goal
    public_vars.motion_vector = [0.5, 0.47];

elseif t < 580
    % Drive straight toward the goal
    public_vars.motion_vector = [0.5, 0.496];

else
    % Stop
    public_vars.motion_vector = [0, 0];
end

end