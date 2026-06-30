% Analyze 12 pure-frequency shapes for Chen-LS and Chen-KF controllers.
%
% This script reads pure-frequency robot raw CSV logs for two controllers.
% It calculates whole-trial metrics directly from the raw data and plots
% comparisons across the 12 shape frequencies.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   Raw CSV files in read_raw_data_dir with X, Y, Time_us, FT1, FT2,
%   v_meas1, v_meas2, beta_hat, K_hat, curvature_hat, and bd_applied.
%
% Outputs:
%   Figures comparing whole-trial metrics and raw signal profiles.
%   Optional CSV table if save_result_table is true.

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_raw_data_dir = "./raw_data/";  % TODO raw data folder
write_result_csv_file = ...
    "./processed_data/pure_frequency_chen_ls_vs_kf_metrics.csv";  % TODO
save_result_table = false;  % TODO true saves result_table to CSV

shape_file_tags = [
    "nu_0"
    "nu_2_fifth"
    "nu_3_fifth"
    "nu_2_third"
    "nu_4_fifth"
    "nu_4_third"
    "nu_3_half"
    "nu_2"
    "nu_5_half"
    "nu_3"
    "nu_4"
    "nu_6"
    ];  % TODO file prefixes

shape_labels = [
    "0"
    "2/5"
    "3/5"
    "2/3"
    "4/5"
    "4/3"
    "3/2"
    "2"
    "5/2"
    "3"
    "4"
    "6"
    ];  % TODO frequency labels for plots

controller_file_suffixes = [
    "chen_var_adm_schunk"
    "chen_kf_var_adm_schunk"
    ];  % TODO raw filename suffixes

controller_names = [
    "Chen-LS"
    "Chen-KF"
    ];  % TODO plot labels

controller_dt = 0.005;          % TODO fallback controller period [s]
min_speed = 0.005;              % TODO minimum speed [m/s]
min_curvature = 0.001;          % TODO minimum curvature [1/m]
max_curvature = 200.0;          % TODO maximum curvature [1/m]
minimum_fit_points = 10;        % TODO minimum samples for power-law fit

beta_min = 0.0;                 % TODO lower beta bound [-]
beta_max = 2.0 / 3.0;           % TODO upper beta bound [-]
trajectory_speed_limits = [0, 0.20];  % TODO speed color range [m/s]
beta_ylim = [0, 0.8];           % TODO beta_hat y range [-]
K_ylim = [0, 0.5];              % TODO K_hat y range
curvature_ylim = [0, 200];      % TODO curvature_hat y range [1/m]
beta_histogram_edges = 0.0:0.01:0.8;  % TODO beta_hat histogram bins
K_histogram_edges = 0.0:0.01:0.5;     % TODO K_hat histogram bins

% -------------------- Read Raw Data And Calculate Metrics --------------------
num_shapes = numel(shape_file_tags);
num_controllers = numel(controller_names);

raw_data = cell(num_shapes, num_controllers);
t_data = cell(num_shapes, num_controllers);

duration_values = nan(num_shapes, num_controllers);
DSJ_values = nan(num_shapes, num_controllers);
mean_force_values = nan(num_shapes, num_controllers);
mean_speed_values = nan(num_shapes, num_controllers);
max_speed_values = nan(num_shapes, num_controllers);
beta_values = nan(num_shapes, num_controllers);
K_values = nan(num_shapes, num_controllers);
R2_values = nan(num_shapes, num_controllers);
RMSE_values = nan(num_shapes, num_controllers);
valid_count_values = nan(num_shapes, num_controllers);

fprintf("\nPure-frequency Chen-LS vs Chen-KF analysis\n");

for shape_idx = 1:num_shapes
    for controller_idx = 1:num_controllers
        read_csv_file = read_raw_data_dir + shape_file_tags(shape_idx) + ...
            "_" + controller_file_suffixes(controller_idx) + ".csv";

        raw_table = readtable(read_csv_file);
        t = make_time_vector(raw_table, controller_dt);

        speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);
        force_xy = sqrt(raw_table.FT1.^2 + raw_table.FT2.^2);

        [beta, K, R2, RMSE, valid_count] = ...
            estimate_power_law_fit_quality( ...
                raw_table.X, raw_table.Y, raw_table.v_meas1, ...
                raw_table.v_meas2, t, min_speed, min_curvature, ...
                max_curvature, minimum_fit_points, beta_min, beta_max);

        raw_data{shape_idx, controller_idx} = raw_table;
        t_data{shape_idx, controller_idx} = t;

        duration_values(shape_idx, controller_idx) = t(end) - t(1);
        DSJ_values(shape_idx, controller_idx) = compute_DSJ(raw_table, t);
        mean_force_values(shape_idx, controller_idx) = mean(force_xy);
        mean_speed_values(shape_idx, controller_idx) = mean(speed_xy);
        max_speed_values(shape_idx, controller_idx) = max(speed_xy);
        beta_values(shape_idx, controller_idx) = beta;
        K_values(shape_idx, controller_idx) = K;
        R2_values(shape_idx, controller_idx) = R2;
        RMSE_values(shape_idx, controller_idx) = RMSE;
        valid_count_values(shape_idx, controller_idx) = valid_count;

        fprintf("%s, %s: beta = %.4f, R2 = %.4f, RMSE = %.4f\n", ...
            shape_labels(shape_idx), controller_names(controller_idx), ...
            beta, R2, RMSE);
    end
end

% -------------------- Create Optional Result Table --------------------
result_table = make_result_table(shape_labels, controller_names, ...
    duration_values, DSJ_values, mean_force_values, mean_speed_values, ...
    max_speed_values, beta_values, K_values, R2_values, RMSE_values, ...
    valid_count_values);

if save_result_table
    writetable(result_table, write_result_csv_file);
    fprintf("\nSaved result table:\n");
    fprintf("%s\n", write_result_csv_file);
end

% -------------------- Plot Whole-Trial Metric Summary --------------------
figure(Name="pure_frequency_summary_metrics", NumberTitle="off");
tiledlayout(3, 3, "TileSpacing", "compact", "Padding", "compact");

plot_grouped_metric(shape_labels, duration_values, controller_names, ...
    "Duration [s]", "Duration");
plot_grouped_metric(shape_labels, DSJ_values, controller_names, ...
    "DSJ [-]", "DSJ");
plot_grouped_metric(shape_labels, mean_force_values, controller_names, ...
    "Mean force [N]", "Mean Force");
plot_grouped_metric(shape_labels, mean_speed_values, controller_names, ...
    "Mean speed [m/s]", "Mean Speed");
plot_grouped_metric(shape_labels, max_speed_values, controller_names, ...
    "Max speed [m/s]", "Max Speed");
plot_grouped_metric(shape_labels, beta_values, controller_names, ...
    "\beta [-]", "Whole-Shape Beta");
plot_grouped_metric(shape_labels, K_values, controller_names, ...
    "K", "Whole-Shape K");
plot_grouped_metric(shape_labels, R2_values, controller_names, ...
    "R2 [-]", "Power-Law R2");
plot_grouped_metric(shape_labels, RMSE_values, controller_names, ...
    "RMSE [-]", "Power-Law RMSE");

% -------------------- Plot Trajectory Speed Heatmaps --------------------
for controller_idx = 1:num_controllers
    figure(Name="pure_frequency_trajectory_" + ...
        controller_names(controller_idx), NumberTitle="off");
    tiledlayout(3, 4, "TileSpacing", "compact", "Padding", "compact");
    colormap(turbo);

    for shape_idx = 1:num_shapes
        raw_table = raw_data{shape_idx, controller_idx};
        speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);

        nexttile;
        scatter(raw_table.X, raw_table.Y, 8, speed_xy, "filled");
        grid on;
        axis equal;
        clim(trajectory_speed_limits);
        xlabel("X [m]");
        ylabel("Y [m]");
        title("\nu = " + shape_labels(shape_idx));
    end

    colorbar;
end

% -------------------- Plot Raw Signal Profiles --------------------
plot_profile_grid(raw_data, t_data, shape_labels, controller_names, ...
    "beta_hat", "\beta_{hat} [-]", beta_ylim, ...
    "pure_frequency_beta_hat_profiles");

plot_profile_grid(raw_data, t_data, shape_labels, controller_names, ...
    "K_hat", "K_{hat}", K_ylim, ...
    "pure_frequency_K_hat_profiles");

plot_profile_grid(raw_data, t_data, shape_labels, controller_names, ...
    "curvature_hat", "\kappa_{hat} [1/m]", curvature_ylim, ...
    "pure_frequency_curvature_hat_profiles");

plot_profile_grid(raw_data, t_data, shape_labels, controller_names, ...
    "bd_applied", "b_d applied [Ns/m]", [NaN, NaN], ...
    "pure_frequency_bd_applied_profiles");

plot_histogram_grid(raw_data, shape_labels, controller_names, ...
    "beta_hat", beta_histogram_edges, "\beta_{hat} [-]", ...
    "pure_frequency_beta_hat_histograms");

plot_histogram_grid(raw_data, shape_labels, controller_names, ...
    "K_hat", K_histogram_edges, "K_{hat}", ...
    "pure_frequency_K_hat_histograms");

plot_speed_profile_grid(raw_data, t_data, shape_labels, controller_names);
plot_force_profile_grid(raw_data, t_data, shape_labels, controller_names);


function t = make_time_vector(raw_table, dt)
    %MAKE_TIME_VECTOR Return time in seconds from Time_us or sample index.

    if ismember("Time_us", string(raw_table.Properties.VariableNames))
        t = (raw_table.Time_us - raw_table.Time_us(1)) * 1e-6;
    else
        t = (0:height(raw_table) - 1)' * dt;
    end
end


function DSJ = compute_DSJ(raw_table, t)
    %COMPUTE_DSJ Compute dimensionless squared jerk from XY velocity.

    movement_time = t(end) - t(1);
    path_step = sqrt(diff(raw_table.X).^2 + diff(raw_table.Y).^2);
    path_length = sum(path_step);

    ax = gradient(raw_table.v_meas1, t);
    ay = gradient(raw_table.v_meas2, t);
    jx = gradient(ax, t);
    jy = gradient(ay, t);

    jerk_squared = jx.^2 + jy.^2;
    DSJ = trapz(t, jerk_squared) * movement_time^5 / path_length^2;
end


function [beta, K, R2, RMSE, valid_count] = ...
    estimate_power_law_fit_quality( ...
        x, y, xd, yd, t, min_speed, min_curvature, max_curvature, ...
        minimum_fit_points, beta_min, beta_max)
    %ESTIMATE_POWER_LAW_FIT_QUALITY Fit v = K*kappa^(-beta).

    ax = gradient(xd, t);
    ay = gradient(yd, t);

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


function result_table = make_result_table(shape_labels, controller_names, ...
    duration_values, DSJ_values, mean_force_values, mean_speed_values, ...
    max_speed_values, beta_values, K_values, R2_values, RMSE_values, ...
    valid_count_values)
    %MAKE_RESULT_TABLE Pack calculated metrics into a table.

    result_shape = strings(0, 1);
    result_controller = strings(0, 1);
    result_duration = [];
    result_DSJ = [];
    result_mean_force = [];
    result_mean_speed = [];
    result_max_speed = [];
    result_beta = [];
    result_K = [];
    result_R2 = [];
    result_RMSE = [];
    result_valid_count = [];

    for shape_idx = 1:numel(shape_labels)
        for controller_idx = 1:numel(controller_names)
            result_shape(end + 1, 1) = shape_labels(shape_idx);
            result_controller(end + 1, 1) = controller_names(controller_idx);
            result_duration(end + 1, 1) = ...
                duration_values(shape_idx, controller_idx);
            result_DSJ(end + 1, 1) = DSJ_values(shape_idx, controller_idx);
            result_mean_force(end + 1, 1) = ...
                mean_force_values(shape_idx, controller_idx);
            result_mean_speed(end + 1, 1) = ...
                mean_speed_values(shape_idx, controller_idx);
            result_max_speed(end + 1, 1) = ...
                max_speed_values(shape_idx, controller_idx);
            result_beta(end + 1, 1) = beta_values(shape_idx, controller_idx);
            result_K(end + 1, 1) = K_values(shape_idx, controller_idx);
            result_R2(end + 1, 1) = R2_values(shape_idx, controller_idx);
            result_RMSE(end + 1, 1) = RMSE_values(shape_idx, controller_idx);
            result_valid_count(end + 1, 1) = ...
                valid_count_values(shape_idx, controller_idx);
        end
    end

    result_table = table(result_shape, result_controller, result_duration, ...
        result_DSJ, result_mean_force, result_mean_speed, result_max_speed, ...
        result_beta, result_K, result_R2, result_RMSE, ...
        result_valid_count, 'VariableNames', ...
        {'frequency', 'controller', 'duration', 'DSJ', 'mean_force', ...
         'mean_speed', 'max_speed', 'beta', 'K', 'R2', 'RMSE', ...
         'valid_count'});
end


function plot_grouped_metric(shape_labels, metric_values, ...
                             controller_names, y_label, plot_title)
    %PLOT_GROUPED_METRIC Plot one grouped bar chart.

    nexttile;
    bar(categorical(shape_labels), metric_values, "grouped");
    grid on;
    xlabel("\nu");
    ylabel(y_label);
    title(plot_title);
    legend(controller_names, "Location", "best");
end


function plot_profile_grid(raw_data, t_data, shape_labels, ...
                           controller_names, column_name, y_label, ...
                           y_limits, figure_name)
    %PLOT_PROFILE_GRID Plot one logged column for all shapes/controllers.

    figure(Name=figure_name, NumberTitle="off");
    tiledlayout(3, 4, "TileSpacing", "compact", "Padding", "compact");

    for shape_idx = 1:numel(shape_labels)
        nexttile;
        hold on;

        for controller_idx = 1:numel(controller_names)
            raw_table = raw_data{shape_idx, controller_idx};
            t = t_data{shape_idx, controller_idx};
            plot(t, raw_table.(column_name), "LineWidth", 1.0);
        end

        grid on;
        xlabel("Time [s]");
        ylabel(y_label);
        title("\nu = " + shape_labels(shape_idx));

        if ~isnan(y_limits(1)) || ~isnan(y_limits(2))
            ylim(y_limits);
        end
    end

    legend(controller_names, "Location", "best");
end


function plot_speed_profile_grid(raw_data, t_data, shape_labels, ...
                                 controller_names)
    %PLOT_SPEED_PROFILE_GRID Plot XY speed profiles for all cases.

    figure(Name="pure_frequency_speed_profiles", NumberTitle="off");
    tiledlayout(3, 4, "TileSpacing", "compact", "Padding", "compact");

    for shape_idx = 1:numel(shape_labels)
        nexttile;
        hold on;

        for controller_idx = 1:numel(controller_names)
            raw_table = raw_data{shape_idx, controller_idx};
            t = t_data{shape_idx, controller_idx};
            speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);
            plot(t, speed_xy, "LineWidth", 1.0);
        end

        grid on;
        xlabel("Time [s]");
        ylabel("Speed XY [m/s]");
        title("\nu = " + shape_labels(shape_idx));
    end

    legend(controller_names, "Location", "best");
end


function plot_histogram_grid(raw_data, shape_labels, controller_names, ...
                             column_name, histogram_edges, x_label, ...
                             figure_name)
    %PLOT_HISTOGRAM_GRID Plot distributions for all shapes/controllers.

    figure(Name=figure_name, NumberTitle="off");
    tiledlayout(3, 4, "TileSpacing", "compact", "Padding", "compact");

    for shape_idx = 1:numel(shape_labels)
        nexttile;
        hold on;

        for controller_idx = 1:numel(controller_names)
            raw_table = raw_data{shape_idx, controller_idx};
            histogram(raw_table.(column_name), histogram_edges, ...
                "Normalization", "probability");
        end

        grid on;
        xlabel(x_label);
        ylabel("Probability");
        title("\nu = " + shape_labels(shape_idx));
        xlim([histogram_edges(1), histogram_edges(end)]);
    end

    legend(controller_names, "Location", "best");
end


function plot_force_profile_grid(raw_data, t_data, shape_labels, ...
                                 controller_names)
    %PLOT_FORCE_PROFILE_GRID Plot external force magnitude profiles.

    figure(Name="pure_frequency_force_profiles", NumberTitle="off");
    tiledlayout(3, 4, "TileSpacing", "compact", "Padding", "compact");

    for shape_idx = 1:numel(shape_labels)
        nexttile;
        hold on;

        for controller_idx = 1:numel(controller_names)
            raw_table = raw_data{shape_idx, controller_idx};
            t = t_data{shape_idx, controller_idx};
            force_xy = sqrt(raw_table.FT1.^2 + raw_table.FT2.^2);
            plot(t, force_xy, "LineWidth", 1.0);
        end

        grid on;
        xlabel("Time [s]");
        ylabel("|F_{xy}| [N]");
        title("\nu = " + shape_labels(shape_idx));
    end

    legend(controller_names, "Location", "best");
end
