% Compare Chen least-squares and Chen Kalman-filter power-law fit quality.
%
% This script reads two raw robot CSV files and calculates the whole-shape
% power-law fit quality for each file.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   Two raw CSV files with columns X, Y, v_meas1, and v_meas2.
%
% Outputs:
%   A small table with beta, K, R2, RMSE, and number of valid samples.
%   A figure comparing R2 and RMSE for Chen-LS and Chen-KF.
%   A 3-by-2 figure comparing beta_hat, K_hat, and curvature_hat profiles.
%   A histogram figure comparing beta_hat distributions.
%   Figures comparing damping, B matrix, intended motion, speed, and force.

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_csv_files = [
    "./raw_data/test_four_leaves_chen_var_adm_schunk.csv"
    "./raw_data/test_four_leaves_chen_kf_var_adm_schunk.csv"
];  % TODO two raw CSV files to compare

controller_names = [
    "Chen-LS"
    "Chen-KF"
];  % TODO labels shown in table and plots

write_result_csv_file = ...
    "./processed_data/chen_ls_vs_kf_fit_quality.csv";  % TODO output CSV

controller_dt = 0.005;          % TODO controller period [s]
min_speed = 0.005;              % TODO minimum speed [m/s]
min_curvature = 0.001;          % TODO minimum curvature [1/m]
max_curvature = 200.0;          % TODO maximum curvature [1/m]
minimum_fit_points = 10;        % TODO minimum samples for power-law fit

beta_min = 0.0;                 % TODO lower beta bound [-]
beta_max = 2.0 / 3.0;           % TODO upper beta bound [-]
beta_histogram_edges = 0.0:0.005:0.7;  % TODO beta_hat histogram bins

% -------------------- Calculate Fit Quality --------------------
beta_values = nan(numel(read_csv_files), 1);
K_values = nan(numel(read_csv_files), 1);
R2_values = nan(numel(read_csv_files), 1);
RMSE_values = nan(numel(read_csv_files), 1);
valid_count_values = nan(numel(read_csv_files), 1);
raw_data = cell(numel(read_csv_files), 1);

fprintf("\nChen-LS vs Chen-KF power-law fit quality\n");

for file_idx = 1:numel(read_csv_files)
    raw_table = readtable(read_csv_files(file_idx));
    raw_data{file_idx} = raw_table;

    x = raw_table.X;
    y = raw_table.Y;
    xd = raw_table.v_meas1;
    yd = raw_table.v_meas2;

    [beta, K, R2, RMSE, valid_count] = estimate_power_law_fit_quality( ...
        x, y, xd, yd, controller_dt, min_speed, min_curvature, ...
        max_curvature, minimum_fit_points, beta_min, beta_max);

    beta_values(file_idx) = beta;
    K_values(file_idx) = K;
    R2_values(file_idx) = R2;
    RMSE_values(file_idx) = RMSE;
    valid_count_values(file_idx) = valid_count;

    fprintf("%s: beta = %.6f, K = %.6f, ", ...
        controller_names(file_idx), beta, K);
    fprintf("R2 = %.6f, RMSE = %.6f, points = %d\n", ...
        R2, RMSE, valid_count);
end

result_table = table(controller_names, read_csv_files, beta_values, ...
    K_values, R2_values, RMSE_values, valid_count_values, ...
    'VariableNames', {'controller_name', 'source_file', 'beta', ...
    'K', 'R2', 'RMSE', 'valid_count'});

% writetable(result_table, write_result_csv_file);

% -------------------- Plot R2 And RMSE --------------------
figure(Name="chen_ls_vs_kf_fit_quality", NumberTitle="off");

subplot(1, 2, 1);
bar(categorical(controller_names), R2_values);
grid on;
xlabel("Controller");
ylabel("R2 [-]");
title("Power-Law R2");
ylim([0, 1]);

subplot(1, 2, 2);
bar(categorical(controller_names), RMSE_values);
grid on;
xlabel("Controller");
ylabel("RMSE [-]");
title("Power-Law RMSE");

% -------------------- Plot Power-Law Profiles --------------------
figure(Name="chen_ls_vs_kf_power_law_profiles", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};
    t = make_time_vector(raw_table, controller_dt);

    subplot(3, 2, file_idx);
    plot(t, raw_table.beta_hat, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("\beta_{hat} [-]");
    title(controller_names(file_idx) + " beta_{hat}");
    ylim([0 0.8]);

    subplot(3, 2, file_idx + 2);
    plot(t, raw_table.K_hat, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("K_{hat}");
    title(controller_names(file_idx) + " K_{hat}");
    ylim([0 0.5]);

    subplot(3, 2, file_idx + 4);
    plot(t, raw_table.curvature_hat, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("\kappa_{hat} [1/m]");
    title(controller_names(file_idx) + " curvature_{hat}");
    ylim([0 200]);
end

% -------------------- Plot Beta Histogram --------------------
figure(Name="chen_ls_vs_kf_beta_hat_histogram", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};

    subplot(1, 2, file_idx);
    histogram(raw_table.beta_hat, beta_histogram_edges);
    grid on;
    xlabel("\beta_{hat} [-]");
    ylabel("Sample count");
    title(controller_names(file_idx) + " beta_{hat} histogram");
    xlim([beta_histogram_edges(1), beta_histogram_edges(end)]);
end

% -------------------- Plot Damping --------------------
figure(Name="chen_ls_vs_kf_damping_profile", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};
    t = make_time_vector(raw_table, controller_dt);

    subplot(1, 2, file_idx);
    plot(t, raw_table.bd_applied, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("b_d applied [Ns/m]");
    title(controller_names(file_idx) + " bd_{applied}");
end

% -------------------- Plot B Matrix --------------------
figure(Name="chen_ls_vs_kf_B_matrix_profiles", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};
    t = make_time_vector(raw_table, controller_dt);

    subplot(4, 2, file_idx);
    plot(t, raw_table.B11, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("B11");
    title(controller_names(file_idx) + " B11");

    subplot(4, 2, file_idx + 2);
    plot(t, raw_table.B12, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("B12");
    title(controller_names(file_idx) + " B12");

    subplot(4, 2, file_idx + 4);
    plot(t, raw_table.B21, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("B21");
    title(controller_names(file_idx) + " B21");

    subplot(4, 2, file_idx + 6);
    plot(t, raw_table.B22, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("B22");
    title(controller_names(file_idx) + " B22");
end

% -------------------- Plot Intended Acceleration And Force --------------------
figure(Name="chen_ls_vs_kf_intended_motion_profiles", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};
    t = make_time_vector(raw_table, controller_dt);

    subplot(4, 2, file_idx);
    plot(t, raw_table.intended_acc_x, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("a_x intended [m/s^2]");
    title(controller_names(file_idx) + " intended acc x");

    subplot(4, 2, file_idx + 2);
    plot(t, raw_table.intended_acc_y, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("a_y intended [m/s^2]");
    title(controller_names(file_idx) + " intended acc y");

    subplot(4, 2, file_idx + 4);
    plot(t, raw_table.intended_force_x, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("F_x intended [N]");
    title(controller_names(file_idx) + " intended force x");

    subplot(4, 2, file_idx + 6);
    plot(t, raw_table.intended_force_y, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("F_y intended [N]");
    title(controller_names(file_idx) + " intended force y");
end

% -------------------- Plot Speed And External Force --------------------
figure(Name="chen_ls_vs_kf_speed_and_force_profiles", NumberTitle="off");

for file_idx = 1:numel(read_csv_files)
    raw_table = raw_data{file_idx};
    t = make_time_vector(raw_table, controller_dt);
    speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);
    force_xy = sqrt(raw_table.FT1.^2 + raw_table.FT2.^2);

    subplot(2, 2, file_idx);
    plot(t, speed_xy, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("Speed XY [m/s]");
    title(controller_names(file_idx) + " speed");

    subplot(2, 2, file_idx + 2);
    plot(t, force_xy, "LineWidth", 1.1);
    grid on;
    xlabel("Time [s]");
    ylabel("|F_{xy}| [N]");
    title(controller_names(file_idx) + " force magnitude");
end

fprintf("\nFit-quality table is available as result_table.\n");
fprintf("To save it, uncomment the writetable line for:\n");
fprintf("%s\n", write_result_csv_file);


function t = make_time_vector(raw_table, dt)
    %MAKE_TIME_VECTOR Return time in seconds from Time_us or sample index.

    if ismember("Time_us", string(raw_table.Properties.VariableNames))
        t = (raw_table.Time_us - raw_table.Time_us(1)) * 1e-6;
    else
        t = (0:height(raw_table) - 1)' * dt;
    end
end


function [beta, K, R2, RMSE, valid_count] = ...
    estimate_power_law_fit_quality( ...
        x, y, xd, yd, dt, min_speed, min_curvature, max_curvature, ...
        minimum_fit_points, beta_min, beta_max)
    %ESTIMATE_POWER_LAW_FIT_QUALITY Fit v = K*kappa^(-beta).
    %
    % Inputs:
    %   x, y - end-effector position [m]
    %   xd, yd - end-effector velocity [m/s]
    %   dt - sampling period [s]
    %
    % Outputs:
    %   beta - fitted power-law exponent [-]
    %   K - fitted velocity coefficient
    %   R2 - coefficient of determination in log space [-]
    %   RMSE - root mean square error in log space
    %   valid_count - number of samples used for the fit

    ax = gradient(xd, dt);
    ay = gradient(yd, dt);

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
