function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)

dt = sampling_period;
L  = kf.interwheel_dist;

vR = u(1); vL = u(2);
v     = (vR + vL) / 2;
omega = (vR - vL) / L;

theta = mu(3);

% Nová poloha (nelineárna funkcia g(x))
new_mu = mu + [v * cos(theta) * dt;
               v * sin(theta) * dt;
               omega * dt];

% Jakobián G (linearizácia g okolo mu)
G = [1, 0, -v * sin(theta) * dt;
     0, 1,  v * cos(theta) * dt;
     0, 0,  1];

% Propagácia kovariancie
new_sigma = G * sigma * G' + kf.R;

end