function [public_vars] = student_workspace(read_only_vars, public_vars)
warning('off', 'all');
public_vars.counter = read_only_vars.counter;

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)

    public_vars = init_kalman_filter(read_only_vars, public_vars);

    z = read_only_vars.gnss_position;
    if ~isempty(z) && all(isfinite(z))
        public_vars.pf_enabled = 0;
        public_vars.particles = [];
    else
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.pf_enabled = 1;
    end

    public_vars.gnss_log = [];
    public_vars.target_idx = 1;
    public_vars.init_spin_done = 0;
    public_vars.path = [];
    public_vars.was_indoor = 0;

end

% Log GNSS data
if ~isempty(read_only_vars.gnss_position)
    public_vars.gnss_log = [public_vars.gnss_log; read_only_vars.gnss_position];
end

% Detect GNSS availability
gnss_available = ~isempty(read_only_vars.gnss_position) && ...
                  all(isfinite(read_only_vars.gnss_position));

if gnss_available
    % Outdoor - EKF
    public_vars.pf_enabled = 0;
    public_vars.was_indoor = 0;
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
else
    % Indoor - PF
    if public_vars.pf_enabled == 0
        % Prve prepnutie do indoor
        public_vars = reinit_pf_from_ekf(read_only_vars, public_vars);
        public_vars.init_spin_done = 0;
        public_vars.path = [];  % zmaž starú cestu
        public_vars.target_idx = 1;
    end
    public_vars.pf_enabled = 1;
    public_vars.was_indoor = 1;
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars);

% Ak PF skonvergoval a EKF je nan, inicializuj EKF z PF
if public_vars.pf_enabled && ~any(isnan(public_vars.estimated_pose)) && any(isnan(public_vars.mu))
    public_vars.mu    = public_vars.estimated_pose';
    public_vars.sigma = eye(3) * 0.5;
end

% 12. Path planning
pose_xy = public_vars.estimated_pose(1:2);
if isempty(public_vars.path) && ~any(isnan(pose_xy))
    % Naplánuj cestu
    public_vars.path = plan_path(read_only_vars, public_vars);
    public_vars.target_idx = 1;
elseif ~isempty(public_vars.path) && ~any(isnan(pose_xy)) && gnss_available
    % Replánuj len keď sme outdoor a príliš ďaleko od cesty
    diffs = public_vars.path - repmat(pose_xy, size(public_vars.path, 1), 1);
    dist_to_path = min(sqrt(sum(diffs.^2, 2)));
    if dist_to_path > 1.5
        public_vars.path = plan_path(read_only_vars, public_vars);
        public_vars.target_idx = 1;
    end
end

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end