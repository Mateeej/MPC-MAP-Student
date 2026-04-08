% function [weights] = weight_particles(particle_measurements, lidar_distances)
% %WEIGHT_PARTICLES Summary of this function goes here
% 
% N = size(particle_measurements, 1);
% weights = ones(N,1) / N;
% 
% end

function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Computes particle weights based on lidar similarity

SIGMA = 0.3;  % expected lidar noise — tune this!
N = size(particle_measurements, 1);
weights = ones(N, 1);

for i = 1:N
    diff = particle_measurements(i,:) - lidar_distances;
    % Ignore Inf values (ray missed all walls)
    valid = isfinite(diff) & isfinite(lidar_distances);
    if any(valid)
        weights(i) = exp(-sum(diff(valid).^2) / (2 * SIGMA^2));
    else
        weights(i) = 1e-10;
    end
end

% Normalize
weights = weights / sum(weights);

end