function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Iterative path smoothing (gradient descent)
%
% Parameters:
%   alpha - how much to pull path toward original waypoints (data weight)
%   beta  - how much to smooth between adjacent waypoints (smooth weight)
%   Higher beta = smoother path but more deviation from original
%   Higher alpha = stays closer to original but less smooth

if isempty(old_path) || size(old_path, 1) < 3
    new_path = old_path;
    return;
end

alpha     = 0.1;   % data weight
beta      = 0.6;   % smooth weight
tolerance = 1e-6;  % convergence threshold
max_iter  = 500;

new_path = old_path;
N        = size(old_path, 1);

for iter = 1:max_iter
    prev_path = new_path;

    for i = 2:N-1
        % Pull toward original
        delta_data   = alpha * (old_path(i,:) - new_path(i,:));
        % Pull toward neighbors
        delta_smooth = beta  * (new_path(i-1,:) + new_path(i+1,:) - 2*new_path(i,:));

        new_path(i,:) = new_path(i,:) + delta_data + delta_smooth;
    end

    % Check convergence
    if max(abs(new_path - prev_path), [], 'all') < tolerance
        break;
    end
end

end