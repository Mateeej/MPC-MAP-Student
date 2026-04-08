% function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
% %COMPUTE_MEASUREMENTS Summary of this function goes here
% 
% measurement = zeros(1, length(lidar_config));
% 
% end

function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_LIDAR_MEASUREMENT Simulates lidar readings for a particle pose

measurement = zeros(1, length(lidar_config));

for i = 1:length(lidar_config)
    % Absolute ray direction = particle heading + lidar angle
    direction = pose(3) + lidar_config(i);
    
    % Cast ray from particle position
    intersections = ray_cast([pose(1), pose(2)], map.walls, direction);
    
    if isempty(intersections)
        measurement(i) = Inf;
    else
        % Distance to closest intersection
        dists = sqrt((intersections(:,1) - pose(1)).^2 + ...
                     (intersections(:,2) - pose(2)).^2);
        measurement(i) = min(dists);
    end
end

end

