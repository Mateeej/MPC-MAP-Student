function path = make_sine(start_point, end_point, amplitude, frequency, invert, n)
% MAKE_SINE Generates a sine wave path between two points
%   start_point - [x, y] počiatočný bod
%   end_point   - [x, y] koncový bod
%   amplitude   - amplitúda (výchylka od priamky)
%   frequency   - počet periód na celej dráhe
%   invert      - true/false (default false)
%   n           - počet bodov (default 50)

if nargin < 5; invert = false; end
if nargin < 6; n = 50; end

dx = end_point(1) - start_point(1);
dy = end_point(2) - start_point(2);
len   = sqrt(dx^2 + dy^2);
angle = atan2(dy, dx);

t = linspace(0, 1, n)';
x_local = t * len;
y_local = amplitude * sin(2*pi * frequency * t);

if invert
    y_local = -y_local;
end

x = start_point(1) + x_local*cos(angle) - y_local*sin(angle);
y = start_point(2) + x_local*sin(angle) + y_local*cos(angle);

path = [x, y];
end