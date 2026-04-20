function [public_vars] = student_workspace(read_only_vars,public_vars)

public_vars.counter = read_only_vars.counter;

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)

    %public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    public_vars.pf_enabled = 0;

    % GNSS log init
    public_vars.gnss_log = [];

    % Path 4: Arc
    seg1 = [2, 2; 2, 2];
    arc1 = make_arc([9, 2], 7, 180, 360, 'cw', 32);
    path_4 = [seg1; arc1];

    public_vars.path = path_4;
    public_vars.target_idx = 1;

end

% Task 1: Log GNSS data
if ~isempty(read_only_vars.gnss_position)
    public_vars.gnss_log = [public_vars.gnss_log; read_only_vars.gnss_position];
end

if mod(read_only_vars.counter, 100) == 0
    gnss_log = public_vars.gnss_log;
    save('algorithms/gnss_data.mat', 'gnss_log');
    fprintf('Saved %d GNSS samples\n', read_only_vars.counter);
end

% 9. Update particle filter
%public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars);

% 12. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end