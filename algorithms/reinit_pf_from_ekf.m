% function public_vars = reinit_pf_from_ekf(read_only_vars, public_vars)
% % Reinicializuj particle filter okolo aktuálnej EKF pozície
% N = read_only_vars.max_particles;
% mu = public_vars.mu;
% 
% SIGMA_XY    = 0.5;  % neistota polohy
% SIGMA_THETA = 0.3;  % neistota orientácie
% 
% x     = mu(1) + randn(N,1) * SIGMA_XY;
% y     = mu(2) + randn(N,1) * SIGMA_XY;
% theta = mu(3) + randn(N,1) * SIGMA_THETA;
% 
% public_vars.particles = [x, y, theta];
% end

function public_vars = reinit_pf_from_ekf(read_only_vars, public_vars)
N = read_only_vars.max_particles;

if ~any(isnan(public_vars.mu))
    mu = public_vars.mu;
    SIGMA_XY    = 1.5;  % väčší rozptyl
    SIGMA_THETA = 0.5;
    x     = mu(1) + randn(N,1) * SIGMA_XY;
    y     = mu(2) + randn(N,1) * SIGMA_XY;
    theta = mu(3) + randn(N,1) * SIGMA_THETA;
else
    limits = read_only_vars.map.limits;
    x     = limits(1) + rand(N,1) * (limits(3) - limits(1));
    y     = limits(2) + rand(N,1) * (limits(4) - limits(2));
    theta = -pi + rand(N,1) * 2*pi;
end

public_vars.particles = [x, y, theta];
end