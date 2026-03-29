% Run this script AFTER collecting data from the simulator.
% Robot stationary (motion_vector = [0,0]) during data collection.
% Requires: algorithms/sensor_data.mat (auto-saved by student_workspace.m)

clear; clc;

%% --- TASK 2 ---
% The robot is equipped with an 8-way LiDAR and a GNSS receiver. 
% Determine the standard deviation (std) sigma for the data from both sensors by placing the robot in a static position (zero velocities) in suitable maps and collecting data for at least 100 simulation periods. 
% Discuss whether the std is consistent across individual LiDAR channels and both GNSS axes. 
% Plot histograms of the measurements.

%% --- TASK 2: Load collected data
load('algorithms/sensor_data.mat');  % loads lidar_log (Nx8) and gnss_log (Nx2)

N_lidar = size(lidar_log, 1);
N_gnss  = size(gnss_log,  1);
fprintf('LiDAR samples: %d\n', N_lidar);
fprintf('GNSS  samples: %d\n', N_gnss);

%% --- TASK 2: Standard Deviation ---

sigma_lidar = std(lidar_log);   % 1x8 — one sigma per channel
sigma_gnss  = std(gnss_log);    % 1x2 — sigma for X and Y

fprintf('\n--- LiDAR sigma per channel (metres) ---\n');
for ch = 1:8
    fprintf('  Channel %d (%.0f deg): sigma = %.4f m\n', ch, (ch-1)*45, sigma_lidar(ch));
end

fprintf('\n--- GNSS sigma ---\n');
fprintf('  X axis: sigma = %.4f m\n', sigma_gnss(1));
fprintf('  Y axis: sigma = %.4f m\n', sigma_gnss(2));

fprintf('\nAre LiDAR sigmas consistent across channels? std of sigmas = %.4f\n', std(sigma_lidar));
fprintf('Are GNSS sigmas consistent across axes?    diff = %.4f\n', abs(sigma_gnss(1) - sigma_gnss(2)));

%% --- TASK 2: Histograms ---
figure('Name', 'LiDAR Histograms', 'NumberTitle', 'off');
for ch = 1:8
    subplot(2, 4, ch);
    histogram(lidar_log(:, ch), 30);
    title(sprintf('LiDAR ch%d (%.0f°)', ch, (ch-1)*45));
    xlabel('Distance (m)');
    ylabel('Count');
    xline(mean(lidar_log(:,ch)), 'r--', 'Mean');
end
sgtitle('LiDAR Measurement Distributions');

figure('Name', 'GNSS Histograms', 'NumberTitle', 'off');
subplot(1, 2, 1);
histogram(gnss_log(:, 1), 30);
title('GNSS X axis');
xlabel('Position (m)'); ylabel('Count');
xline(mean(gnss_log(:,1)), 'r--', 'Mean');

subplot(1, 2, 2);
histogram(gnss_log(:, 2), 30);
title('GNSS Y axis');
xlabel('Position (m)'); ylabel('Count');
xline(mean(gnss_log(:,2)), 'r--', 'Mean');
sgtitle('GNSS Measurement Distributions');

%% --- TASK 3 ---
% Use the measurements from the previous step and MATLAB's internal cov function to assemble the covariance matrix for both sensors.
% Verify that the resultant matrix is of size 8×8 for the LiDAR and 2×2 for the GNSS.
% Ensure that the values on the main diagonal are equal to sigma^2, i.e., variance=std^2`.

%% --- TASK 3: Covariance Matrices ---

C_lidar = cov(lidar_log);   % 8x8
C_gnss  = cov(gnss_log);    % 2x2

fprintf('\n--- LiDAR Covariance Matrix (8x8) ---\n');
disp(C_lidar);

fprintf('--- GNSS Covariance Matrix (2x2) ---\n');
disp(C_gnss);

% Verify diagonal equals sigma^2
fprintf('Verify LiDAR diagonal == sigma^2 (should be ~0):\n');
disp(max(abs(diag(C_lidar)' - sigma_lidar.^2)));

fprintf('Verify GNSS diagonal == sigma^2 (should be ~0):\n');
disp(max(abs(diag(C_gnss)' - sigma_gnss.^2)));

%% --- TASK 4 ---
% Create a function norm_pdf to assemble the probability density function (pdf) of the normal distribution.
% The function should accept three arguments: x (values at which to evaluate the pdf), mu (mean), and sigma (standard deviation).
% Utilize this function along with the sigma values from Task 2 (e.g., for the 1st LiDAR channel and the X GNSS axis) to generate two pdf illustrating the noise characteristics of the robot's sensors.
% Plot them in a single image (use mu=0 in both cases).

%% --- TASK 4: Normal Distribution PDF -

function y = norm_pdf(x, mu, sigma)
    y = (1 / (sigma * sqrt(2*pi))) .* exp(-0.5 * ((x - mu) / sigma).^2);
end

x = linspace(-0.5, 0.5, 1000);  

y_lidar = norm_pdf(x, 0, sigma_lidar(1));  % channel 1
y_gnss  = norm_pdf(x, 0, sigma_gnss(1));   % X axis

figure('Name', 'Sensor Noise PDFs', 'NumberTitle', 'off');
hold on;
plot(x, y_lidar, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('LiDAR ch1 (\\sigma=%.4f)', sigma_lidar(1)));
plot(x, y_gnss,  'r-', 'LineWidth', 2, 'DisplayName', sprintf('GNSS X   (\\sigma=%.4f)', sigma_gnss(1)));
xlabel('Error (m)');
ylabel('Probability Density');
title('Normal Distribution PDF of Sensor Noise (\mu = 0)');
legend('Location', 'best');
grid on;
hold off;