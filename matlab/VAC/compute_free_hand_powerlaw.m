% Compute whole-shape power-law parameters from free-hand drawing data.
%
% This script reads timestamped free-hand position samples and estimates one
% velocity-curvature power law for each drawing.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   read_free_hand_data_dir: folder with x, y, time_s CSV files
%
% Outputs:
%   write_powerlaw_csv_file: table with beta, K, R2, and RMSE
%   Figure:
%   1) calculated beta by free-hand shape

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_free_hand_data_dir = "./raw_data/free_hand/";  % TODO input folder
write_powerlaw_csv_file = ...                       % TODO output CSV
    "./processed_data/free_hand_powerlaw.csv";

shape_ids = [1, 2, 3, 4];  % TODO [eight, ellipse, clover, squircle]
shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

min_speed = 0.001;             % TODO minimum speed [data unit/s]
min_curvature = 0.001;         % TODO minimum curvature [1/data unit]
max_curvature = 200.0;         % TODO maximum curvature [1/data unit]
minimum_fit_points = 10;       % TODO minimum samples for LS fit

beta_min = 0.0;                % TODO lower beta bound
beta_max = 2.0 / 3.0;          % TODO upper beta bound

% -------------------- Estimate Whole-Shape Power Law --------------------
fprintf("\nFree-hand whole-shape power-law estimates\n");

beta_values = nan(numel(shape_names), 1);
K_values = nan(numel(shape_names), 1);
R2_values = nan(numel(shape_names), 1);
RMSE_values = nan(numel(shape_names), 1);
valid_count_values = nan(numel(shape_names), 1);
duration_values = nan(numel(shape_names), 1);

for shape_idx = 1:numel(shape_names)
    read_csv_file = read_free_hand_data_dir + shape_names(shape_idx) + ...
        ".csv";

    raw_table = readtable(read_csv_file);

    x = raw_table.x;
    y = raw_table.y;
    time_s = raw_table.time_s;

    valid_sample_idx = isfinite(x) & isfinite(y) & isfinite(time_s);
    x = x(valid_sample_idx);
    y = y(valid_sample_idx);
    time_s = time_s(valid_sample_idx);
    time_s = time_s - time_s(1);

    [beta, K, R2, RMSE, valid_count] = ...
        estimate_free_hand_power_law( ...
            x, y, time_s, min_speed, min_curvature, max_curvature, ...
            minimum_fit_points, beta_min, beta_max);

    beta_values(shape_idx) = beta;
    K_values(shape_idx) = K;
    R2_values(shape_idx) = R2;
    RMSE_values(shape_idx) = RMSE;
    valid_count_values(shape_idx) = valid_count;
    duration_values(shape_idx) = time_s(end) - time_s(1);

    fprintf("%s: beta = %.6f, K = %.6f, ", shape_names(shape_idx), ...
        beta, K);
    fprintf("R2 = %.6f, RMSE = %.6f, points = %d\n", ...
        R2, RMSE, valid_count);
end

% -------------------- Save Results --------------------
powerlaw_table = table(shape_ids(:), shape_names(:), beta_values, ...
    K_values, R2_values, RMSE_values, valid_count_values, ...
    duration_values, 'VariableNames', ...
    {'shape_id', 'shape_name', 'beta', 'K', 'R2', 'RMSE', ...
     'valid_count', 'duration_s'});

writetable(powerlaw_table, write_powerlaw_csv_file);

fprintf("\nFree-hand power-law results written to:\n");
fprintf("%s\n", write_powerlaw_csv_file);

% -------------------- Plot Calculated Beta --------------------
figure(Name="free_hand_beta_by_shape", NumberTitle="off");
bar(categorical(shape_names), beta_values);
grid on;

xlabel("Shape");
ylabel("\beta [-]");
title("Free-Hand Calculated Beta by Shape");


function [beta, K, R2, RMSE, valid_count] = ...
    estimate_free_hand_power_law( ...
        x, y, time_s, min_speed, min_curvature, max_curvature, ...
        minimum_fit_points, beta_min, beta_max)
    %ESTIMATE_FREE_HAND_POWER_LAW Fit v = K*kappa^(-beta).
    %
    % Inputs:
    %   x, y - free-hand drawing position [data unit]
    %   time_s - timestamp starting from zero [s]
    %
    % Outputs:
    %   beta - fitted power-law exponent [-]
    %   K - fitted velocity coefficient
    %   R2 - coefficient of determination of log-space fit [-]
    %   RMSE - root mean square error of log-space fit
    %   valid_count - number of samples used in the fit

    xd = gradient(x, time_s);
    yd = gradient(y, time_s);

    ax = gradient(xd, time_s);
    ay = gradient(yd, time_s);

    speed = sqrt(xd.^2 + yd.^2);
    curvature = abs(xd .* ay - yd .* ax) ./ (speed.^3);

    valid_idx = speed >= min_speed & curvature >= min_curvature;

    speed = speed(valid_idx);
    curvature = curvature(valid_idx);
    curvature = min(max(curvature, min_curvature), max_curvature);

    log_curvature = log(curvature);
    log_speed = log(speed);
    valid_count = numel(log_speed);

    beta = NaN;
    K = NaN;
    R2 = NaN;
    RMSE = NaN;

    if valid_count < minimum_fit_points
        return;
    end

    fit_coeff = polyfit(log_curvature, log_speed, 1);
    slope = fit_coeff(1);
    intercept = fit_coeff(2);

    fitted_log_speed = polyval(fit_coeff, log_curvature);
    residual = log_speed - fitted_log_speed;

    sum_squared_error = sum(residual.^2);
    total_squared_error = sum((log_speed - mean(log_speed)).^2);

    beta = min(max(-slope, beta_min), beta_max);
    K = exp(intercept);
    R2 = 1.0 - sum_squared_error / total_squared_error;
    RMSE = sqrt(mean(residual.^2));
end
