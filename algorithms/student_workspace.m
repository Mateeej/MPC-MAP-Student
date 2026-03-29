function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    % Clear any leftover data file from previous run
    if isfile('algorithms/sensor_data.mat')
        delete('algorithms/sensor_data.mat');
    end
    
    public_vars.lidar_log = [];
    public_vars.gnss_log  = [];

end

% Task 2: Log sensor data every tick
public_vars.lidar_log = [public_vars.lidar_log; read_only_vars.lidar_distances];
public_vars.gnss_log  = [public_vars.gnss_log;  read_only_vars.gnss_position];
 
% Task 2: Save data to file every 100 ticks
if mod(read_only_vars.counter, 100) == 0
    lidar_log = public_vars.lidar_log;
    gnss_log  = public_vars.gnss_log;
    save('algorithms/sensor_data.mat', 'lidar_log', 'gnss_log');
    fprintf('Saved %d samples to sensor_data.mat\n', read_only_vars.counter);
end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

