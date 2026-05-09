function [estimated_pose] = estimate_pose(public_vars)

if public_vars.pf_enabled && ~isempty(public_vars.particles) && public_vars.counter > 30
    % Indoor: particle filter
    %estimated_pose = median(public_vars.particles);
    % Váhovaný priemer pre theta (circular mean)
    sin_mean = mean(sin(public_vars.particles(:,3)));
    cos_mean = mean(cos(public_vars.particles(:,3)));
    theta = atan2(sin_mean, cos_mean);
    estimated_pose = [median(public_vars.particles(:,1)), ...
                    median(public_vars.particles(:,2)), ...
                    theta];
elseif ~isempty(public_vars.mu) && all(isfinite(public_vars.mu))
    % Outdoor: EKF
    estimated_pose = public_vars.mu';
else
    estimated_pose = nan(1, 3);
end

end