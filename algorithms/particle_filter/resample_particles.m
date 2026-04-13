function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Systematic resampling algorithm

N = size(particles, 1);
new_particles = zeros(N, size(particles, 2));

% Cumulative sum of weights
cumulative = cumsum(weights);
cumulative(end) = 1.0;

% Random starting point
r = rand / N;
j = 1;

for i = 1:N
    threshold = r + (i-1) / N;
    while cumulative(j) < threshold
        j = j + 1;
    end
    new_particles(i,:) = particles(j,:);
end

end

