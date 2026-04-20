% function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
% 
% % C: measurement matrix - GNSS meria len x,y (nie theta)
% public_vars.kf.C = [1, 0, 0;
%                     0, 1, 0];
% 
% % Q: measurement noise covariance (z GNSS dát - Task 1)
% public_vars.kf.Q = [0.1929, 0.0142;
%                     0.0142, 0.2924];
% 
% % R: process noise covariance (3x3) - tune neskôr
% public_vars.kf.R = diag([0.01, 0.01, 0.01]);
% 
% % interwheel distance pre ekf_predict
% public_vars.kf.interwheel_dist = read_only_vars.agent_drive.interwheel_dist;
% 
% % Počiatočný stav - Task 3: známa počiatočná poloha
% public_vars.mu    = [2; 2; pi/2];
% public_vars.sigma = zeros(3, 3);
% 
% end

function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

% C: measurement matrix - GNSS meria len x,y (nie theta)
public_vars.kf.C = [1, 0, 0;
                    0, 1, 0];

% Q: measurement noise covariance (z GNSS dát - Task 1)
public_vars.kf.Q = [0.1929, 0.0142;
                    0.0142, 0.2924];


% R: process noise covariance (3x3)
public_vars.kf.R = diag([0.005, 0.005, 0.005]);

% interwheel distance
public_vars.kf.interwheel_dist = read_only_vars.agent_drive.interwheel_dist;

% Task 4: Neznáma počiatočná poloha - použij GNSS meranie
z = read_only_vars.gnss_position;
public_vars.mu = [z(1); z(2); pi/2];  % theta neznáma - odhadneme pi/2

% Vysoká neistota pre theta, stredná pre x,y
% public_vars.sigma = [0.1929, 0.0142, 0;
%                      0.0142, 0.2924, 0;
%                      0,      0,      9.0];  % vysoký rozptyl pre theta

public_vars.sigma = [0.1929, 0.0142, 0;
                     0.0142, 0.2924, 0;
                     0,      0,      1.0];

end