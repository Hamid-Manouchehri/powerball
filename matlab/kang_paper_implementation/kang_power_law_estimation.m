% Estimate velocity-curvature power-law parameters from one Schunk dataset.
%
% This script mirrors estimate_power_law_from_history() in
% src_main/chen_var_adm.cpp.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   read_csv_file: Schunk CSV log with Time_us, X, Y, v_meas1, v_meas2.
%
% Outputs:
%   Command window:
%   1) final estimated beta_hat and K_hat
%   2) number of valid least-squares windows
%   Figures:
%   1) beta_hat, K_hat, and curvature_hat versus time
%   2) final-window log-log least-squares fit

clc;
clear;
close all;

addpath("functions/");

% -------------------- User Settings --------------------
read_csv_file = ...
    "../chen_paper_implementation/raw_data/" + ...
    "eight_sign_kang_indirect_var_adm_schunk.csv";
    % TODO dataset CSV

controller_dt = 0.005;        % TODO controller period [s]
power_law_window = 50;        % TODO history window N
power_law_min_points = 10;    % TODO least-square minimum samples

min_speed_for_guidance = 0.005;  % TODO minimum speed [m/s]
min_curvature = 0.001;           % TODO minimum curvature [1/m]
max_curvature = 200.0;           % TODO maximum curvature [1/m]

beta_min = 0.0;           % paper lower bound
beta_max = 2.0 / 3.0;     % paper upper bound

% -------------------- Read Dataset --------------------
schunk_table = readtable(read_csv_file);

t = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
x = schunk_table.X;
y = schunk_table.Y;
xd = schunk_table.v_meas1;
yd = schunk_table.v_meas2;

% -------------------- Estimate Moving Power Law --------------------
[beta_hat, K_hat, curvature_hat, indirect_ready] = ...
    estimate_power_law_over_time( ...
        xd, yd, controller_dt, power_law_window, ...
        power_law_min_points, min_speed_for_guidance, ...
        min_curvature, max_curvature, beta_min, beta_max);

valid_idx = find(indirect_ready);

fprintf("\nPower-law estimate from dataset:\n");
fprintf("%s\n", read_csv_file);
fprintf("Valid least-square windows = %d\n", numel(valid_idx));

if ~isempty(valid_idx)
    final_idx = valid_idx(end);

    fprintf("Final beta_hat = %.6f\n", beta_hat(final_idx));
    fprintf("Final K_hat = %.6f\n", K_hat(final_idx));
    fprintf("Final curvature_hat = %.6f 1/m\n", curvature_hat(final_idx));
end

% -------------------- Plot Estimated Signals --------------------
figure(Name="power_law_estimation", NumberTitle="off");

subplot(3,1,1);
builtin('plot', t, beta_hat, 'LineWidth', 1.2);
grid on;
ylabel("\beta [-]");
title("Estimated power-law parameters");

subplot(3,1,2);
builtin('plot', t, K_hat, 'LineWidth', 1.2);
grid on;
ylabel("K");

subplot(3,1,3);
builtin('plot', t, curvature_hat, 'LineWidth', 1.2);
grid on;
xlabel("Time [s]");
ylabel("\kappa [1/m]");

if ~isempty(valid_idx)
    % -------------------- Plot Final Window Fit --------------------
    [log_curvature, log_speed, fit_line] = get_final_window_fit( ...
        xd, yd, final_idx, controller_dt, power_law_window, ...
        min_speed_for_guidance, min_curvature, max_curvature);

    figure(Name="power_law_final_window_fit", NumberTitle="off");
    scatter(log_curvature, log_speed, 24, 'filled');
    hold on;
    builtin('plot', log_curvature, fit_line, 'LineWidth', 1.5);
    grid on;
    xlabel("log(\kappa)");
    ylabel("log(v)");
    title("Final-window least-squares fit");
    legend("Samples", "Least-squares fit", "Location", "best");
end


function [beta_hat, K_hat, curvature_hat, indirect_ready] = ...
    estimate_power_law_over_time( ...
        xd, yd, dt, power_law_window, power_law_min_points, ...
        min_speed_for_guidance, min_curvature, max_curvature, ...
        beta_min, beta_max)
    %ESTIMATE_POWER_LAW_OVER_TIME Estimate beta and K over moving windows.
    %
    % Inputs:
    %   xd, yd - planar end-effector velocity [m/s]
    %   dt - controller period [s]
    %   power_law_window - number of history samples used by C++
    %   power_law_min_points - minimum valid LS samples
    %
    % Outputs:
    %   beta_hat - estimated power-law exponent [-]
    %   K_hat - estimated velocity coefficient
    %   curvature_hat - latest valid curvature in each window [1/m]
    %   indirect_ready - true if the least-square fit was valid

    num_samples = numel(xd);
    beta_hat = nan(num_samples, 1);
    K_hat = nan(num_samples, 1);
    curvature_hat = nan(num_samples, 1);
    indirect_ready = false(num_samples, 1);

    history_size = power_law_window + 2;

    for sample_idx = 1:num_samples
        first_idx = max(1, sample_idx - history_size + 1);

        xd_hist = xd(first_idx:sample_idx);
        yd_hist = yd(first_idx:sample_idx);

        [beta_now, K_now, curvature_now, ready_now] = ...
            estimate_power_law_from_history_matlab( ...
                xd_hist, yd_hist, dt, power_law_min_points, ...
                min_speed_for_guidance, min_curvature, ...
                max_curvature, beta_min, beta_max);

        beta_hat(sample_idx) = beta_now;
        K_hat(sample_idx) = K_now;
        curvature_hat(sample_idx) = curvature_now;
        indirect_ready(sample_idx) = ready_now;
    end
end


function [beta_hat, K_hat, curvature_hat, indirect_ready] = ...
    estimate_power_law_from_history_matlab( ...
        xd_hist, yd_hist, dt, power_law_min_points, ...
        min_speed_for_guidance, min_curvature, max_curvature, ...
        beta_min, beta_max)
    %ESTIMATE_POWER_LAW_FROM_HISTORY_MATLAB Least-squares power-law fit.
    %
    % This is the MATLAB version of estimate_power_law_from_history()
    % in src_main/chen_var_adm.cpp.

    history_size = numel(xd_hist);

    sum_x = 0.0;
    sum_y = 0.0;
    sum_xx = 0.0;
    sum_xy = 0.0;
    count = 0;

    beta_hat = nan;
    K_hat = nan;
    curvature_hat = nan;
    indirect_ready = false;

    for hist_idx = 2:history_size - 1
        v_prev = [xd_hist(hist_idx - 1), yd_hist(hist_idx - 1)];
        v = [xd_hist(hist_idx), yd_hist(hist_idx)];
        v_next = [xd_hist(hist_idx + 1), yd_hist(hist_idx + 1)];

        a = (v_next - v_prev) / (2.0 * dt);

        speed = norm(v);
        if speed < min_speed_for_guidance
            continue;
        end

        curvature = abs(vec2_cross(v, a)) / (speed * speed * speed);
        if curvature < min_curvature
            continue;
        end

        curvature = clamp_value(curvature, min_curvature, max_curvature);
        curvature_hat = curvature;

        log_curvature = log(curvature);
        log_speed = log(speed);

        sum_x = sum_x + log_curvature;
        sum_y = sum_y + log_speed;
        sum_xx = sum_xx + log_curvature * log_curvature;
        sum_xy = sum_xy + log_curvature * log_speed;
        count = count + 1;
    end

    if count < power_law_min_points
        return;
    end

    denom = count * sum_xx - sum_x * sum_x;
    if abs(denom) < 1e-6
        return;
    end

    slope = (count * sum_xy - sum_x * sum_y) / denom;
    intercept = (sum_y - slope * sum_x) / count;

    beta_hat = clamp_value(-slope, beta_min, beta_max);
    K_hat = exp(intercept);
    indirect_ready = true;
end


function [log_curvature, log_speed, fit_line] = get_final_window_fit( ...
    xd, yd, final_idx, dt, power_law_window, min_speed_for_guidance, ...
    min_curvature, max_curvature)
    %GET_FINAL_WINDOW_FIT Return log-log samples and fit for final window.

    history_size = power_law_window + 2;
    first_idx = max(1, final_idx - history_size + 1);

    xd_hist = xd(first_idx:final_idx);
    yd_hist = yd(first_idx:final_idx);

    log_curvature = [];
    log_speed = [];

    for hist_idx = 2:numel(xd_hist) - 1
        v_prev = [xd_hist(hist_idx - 1), yd_hist(hist_idx - 1)];
        v = [xd_hist(hist_idx), yd_hist(hist_idx)];
        v_next = [xd_hist(hist_idx + 1), yd_hist(hist_idx + 1)];

        a = (v_next - v_prev) / (2.0 * dt);

        speed = norm(v);
        if speed < min_speed_for_guidance
            continue;
        end

        curvature = abs(vec2_cross(v, a)) / (speed * speed * speed);
        if curvature < min_curvature
            continue;
        end

        curvature = clamp_value(curvature, min_curvature, max_curvature);
        log_curvature(end + 1, 1) = log(curvature); %#ok<SAGROW>
        log_speed(end + 1, 1) = log(speed); %#ok<SAGROW>
    end

    coeff = polyfit(log_curvature, log_speed, 1);
    fit_line = polyval(coeff, log_curvature);
end


function cross_value = vec2_cross(a, b)
    %VEC2_CROSS 2D scalar cross product.
    cross_value = a(1) * b(2) - a(2) * b(1);
end


function y = clamp_value(x, lower_bound, upper_bound)
    %CLAMP_VALUE Limit x to [lower_bound, upper_bound].
    y = min(max(x, lower_bound), upper_bound);
end
