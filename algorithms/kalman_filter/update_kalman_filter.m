function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

mu    = public_vars.mu;
sigma = public_vars.sigma;

% Skip ak mu je nan (indoor bez inicializacie)
if isempty(mu) || any(isnan(mu))
    return;
end

% I. Prediction
u = public_vars.motion_vector;
if isempty(u)
    u = [0, 0];
end
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement correction
z = read_only_vars.gnss_position;
if ~isempty(z) && all(isfinite(z))
    [mu, sigma] = kf_measure(mu, sigma, z', public_vars.kf);
end

end