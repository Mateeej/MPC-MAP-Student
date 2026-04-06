% MAKE_ARC Generates a circular arc as a sequence of waypoints
%   center      - [cx, cy] stred kružnice
%   r           - polomer
%   angle_start - počiatočný uhol (stupne)
%   angle_end   - koncový uhol (stupne)
%   n           - počet bodov (default 15)

function arc = make_arc(center, r, angle_start, angle_end, direction, n)
% direction: 'cw' or 'ccw'
if nargin < 5; direction = 'ccw'; end
if nargin < 6; n = 15; end

if strcmp(direction, 'cw')
    if angle_end > angle_start
        angle_end = angle_end - 360;
    end
else % ccw
    if angle_end < angle_start
        angle_end = angle_end + 360;
    end
end

t = linspace(deg2rad(angle_start), deg2rad(angle_end), n)';
arc = [center(1) + r*cos(t), center(2) + r*sin(t)];
end