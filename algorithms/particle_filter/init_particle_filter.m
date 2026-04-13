function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Initializes particles at random poses within map limits

N = read_only_vars.max_particles;  % 1000 particles

% Get map limits [xMin, yMin, xMax, yMax]
limits = read_only_vars.map.limits;
xMin = limits(1); xMax = limits(3);
yMin = limits(2); yMax = limits(4);

% Generate random particles [x, y, theta]
x     = xMin + rand(N,1) * (xMax - xMin);
y     = yMin + rand(N,1) * (yMax - yMin);
theta = -pi  + rand(N,1) * 2*pi;

public_vars.particles = [x, y, theta];

end

