function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)

% C - measurement matrix (2x3): extracts x,y from state [x,y,theta]
C = kf.C;
Q = kf.Q;

% Kalman gain
K = sigma * C' / (C * sigma * C' + Q);

% Update
new_mu    = mu + K * (z - C * mu);
new_sigma = (eye(3) - K * C) * sigma;

end