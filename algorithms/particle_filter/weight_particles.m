% function [weights] = weight_particles(particle_measurements, lidar_distances)
% 
% SIGMA = 0.5;      % väčší = tolerantnejší
% MIN_W = 0.0001;   % minimum weight pre každú časticu
% N = size(particle_measurements, 1);
% weights = ones(N, 1);
% 
% for i = 1:N
%     diff = particle_measurements(i,:) - lidar_distances;
%     valid = isfinite(lidar_distances);
%     if any(valid)
%         d = diff(valid);
%         inf_penalty = sum(~isfinite(particle_measurements(i, valid))) * 1.0;
%         weights(i) = exp(-(sum(d(isfinite(d)).^2) + inf_penalty) / (2 * SIGMA^2));
%         weights(i) = max(weights(i), MIN_W);  % minimum weight
%     else
%         weights(i) = MIN_W;
%     end
% end
% 
% weights = weights / sum(weights);
% 
% end

function [weights] = weight_particles(particle_measurements, lidar_distances)

MIN_W = 0.000001;
N = size(particle_measurements, 1);
weights = ones(N, 1);

for i = 1:N
    diff = particle_measurements(i,:) - lidar_distances;
    valid = isfinite(lidar_distances);
    if any(valid)
        d = diff(valid);
        % Adaptivny SIGMA - kratke vzdialenosti = prisnejsi filter
        avg_dist = mean(lidar_distances(valid));
        sigma_adaptive = max(0.1, min(0.5, avg_dist * 0.1));
        
        inf_penalty = sum(~isfinite(particle_measurements(i, valid))) * 1.0;
        weights(i) = exp(-(sum(d(isfinite(d)).^2) + inf_penalty) / (2 * sigma_adaptive^2));
        weights(i) = max(weights(i), MIN_W);
    else
        weights(i) = MIN_W;
    end
end

weights = weights / sum(weights);

end