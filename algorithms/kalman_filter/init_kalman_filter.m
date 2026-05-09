function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

% C: measurement matrix - GNSS meria len x,y
public_vars.kf.C = [1, 0, 0;
                    0, 1, 0];

% Q: GNSS measurement noise
public_vars.kf.Q = [0.1929, 0.0142;
                    0.0142, 0.2924];

% R: process noise
public_vars.kf.R = diag([0.005, 0.005, 0.005]);

% interwheel distance
public_vars.kf.interwheel_dist = read_only_vars.agent_drive.interwheel_dist;

% Initial state
z = read_only_vars.gnss_position;
if ~isempty(z) && all(isfinite(z))
    % Outdoor: inicializuj z GNSS
    public_vars.mu    = [z(1); z(2); pi/2];
    public_vars.sigma = [0.1929, 0.0142, 0;
                         0.0142, 0.2924, 0;
                         0,      0,      9.0];
else
    % Indoor: neznama poloha - PF preberie
    public_vars.mu    = [nan; nan; nan];
    public_vars.sigma = eye(3) * 9.0;
end

end