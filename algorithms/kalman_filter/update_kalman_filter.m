% function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
% %UPDATE_KALMAN_FILTER Summary of this function goes here
% 
% mu = public_vars.mu;
% sigma = public_vars.sigma;
% 
% % I. Prediction
% u = [];
% [mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);
% 
% % II. Measurement
% z = [];
% [mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);
% 
% end

function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

mu    = public_vars.mu;
sigma = public_vars.sigma;

if isempty(mu) || isempty(sigma)
    return;
end

% I. Prediction - control input [vR, vL]
u = public_vars.motion_vector;
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement - GNSS [x, y]
z = read_only_vars.gnss_position;
if ~isempty(z) && all(isfinite(z))
    [mu, sigma] = kf_measure(mu, sigma, z', public_vars.kf);
end

end
