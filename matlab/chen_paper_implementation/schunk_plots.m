% Compare Schunk shape datasets from Chen/Kang variable admittance tests.
%
% Inputs:
%   read_csv_files: CSV logs with Time_us, X, Y, FT, and velocity columns.
%
% Outputs:
%   Figure 1:
%   1) overlaid Cartesian end-effector trajectories [m]
%   Figure 2:
%   1) dimensionless squared jerk (DSJ)
%   2) mean planar force [N]
%   Figure 3:
%   1) beta exponent [-]
%   2) K parameter, if available
%   3) damping, if available
%   Command window:
%   1) DSJ, mean force, path length, and movement time for each dataset

clc;
clear;
close all;

addpath("functions/");

% -------------------- User Settings --------------------
read_csv_files = [
    "raw_data/" + "eight_sign_chen_var_adm_schunk.csv"             % TODO
    "raw_data/" + "eight_sign_kang_indirect_var_adm_schunk.csv"    % TODO
    "raw_data/" + "ellipse_chen_var_adm_schunk.csv"                % TODO
    "raw_data/" + "ellipse_kang_indirect_var_adm_schunk.csv"       % TODO
    "raw_data/" + "squircle_chen_var_adm_schunk.csv"               % TODO
    "raw_data/" + "squircle_kang_indirect_var_adm_schunk.csv"      % TODO
    "raw_data/" + "four_leaves_chen_var_adm_schunk.csv"            % TODO
    "raw_data/" + "four_leaves_kang_indirect_var_adm_schunk.csv"   % TODO
];

plot_labels = [
    "Eight Chen"
    "Eight Kang"
    "Ellipse Chen"
    "Ellipse Kang"
    "Squircle Chen"
    "Squircle Kang"
    "clovers Chen"
    "clovers Kang"
];

use_robot_xy_transform = false;  % TODO true if you want aligned plots
robot_offset_x = 0.0;            % TODO robot x offset [m]
robot_offset_y = 0.0;            % TODO robot y offset [m]
robot_rotation_deg = 0.0;        % TODO robot planar rotation [deg]

% -------------------- Read And Compute Metrics --------------------
num_files = numel(read_csv_files);
shape_data = cell(num_files, 1);

for file_idx = 1:num_files
    shape_data{file_idx} = read_shape_dataset( ...
        read_csv_files(file_idx), ...
        plot_labels(file_idx), ...
        use_robot_xy_transform, ...
        robot_offset_x, ...
        robot_offset_y, ...
        robot_rotation_deg);
end

fprintf("\nShape comparison metrics\n");
fprintf("%-18s %14s %14s %14s %14s\n", ...
    "Dataset", "DSJ", "Mean F [N]", "Path [m]", "Time [s]");

for file_idx = 1:num_files
    fprintf("%-18s %14.6g %14.6f %14.6f %14.6f\n", ...
        char(shape_data{file_idx}.label), ...
        shape_data{file_idx}.dsj, ...
        shape_data{file_idx}.mean_force, ...
        shape_data{file_idx}.path_length, ...
        shape_data{file_idx}.movement_time);
end

% -------------------- Plot Trajectories --------------------
figure(Name="shape_trajectory_comparison", NumberTitle="off");
hold on;
grid on;
axis equal;

colors = lines(num_files);

for file_idx = 1:num_files
    builtin('plot', ...
        shape_data{file_idx}.x, ...
        shape_data{file_idx}.y, ...
        'LineWidth', 1.3, ...
        'Color', colors(file_idx, :));
end

xlabel("X [m]");
ylabel("Y [m]");
title("End-effector trajectory comparison");
legend(plot_labels, "Interpreter", "none", "Location", "bestoutside");

% -------------------- Plot Scalar Metrics --------------------
dsj_values = zeros(num_files, 1);
mean_force_values = zeros(num_files, 1);

for file_idx = 1:num_files
    dsj_values(file_idx) = shape_data{file_idx}.dsj;
    mean_force_values(file_idx) = shape_data{file_idx}.mean_force;
end

figure(Name="shape_metric_comparison", NumberTitle="off");

subplot(2,1,1);
bar(dsj_values);
grid on;
ylabel("DSJ [-]");
title("Dimensionless squared jerk");
set(gca, "XTick", 1:num_files, "XTickLabel", plot_labels);
xtickangle(35);

subplot(2,1,2);
bar(mean_force_values);
grid on;
ylabel("Mean force [N]");
title("Mean planar interaction force");
set(gca, "XTick", 1:num_files, "XTickLabel", plot_labels);
xtickangle(35);

% -------------------- Plot Controller Signals --------------------
figure(Name="controller_signal_comparison", NumberTitle="off");

for file_idx = 1:num_files
    subplot(num_files, 3, 3 * (file_idx - 1) + 1);
    builtin('plot', ...
        shape_data{file_idx}.t, ...
        shape_data{file_idx}.beta, ...
        'LineWidth', 1.0, ...
        'Color', colors(file_idx, :));
    grid on;
    ylabel(plot_labels(file_idx), "Interpreter", "none");
    if file_idx == 1
        title("\beta [-]");
    end
    if file_idx == num_files
        xlabel("Time [s]");
    end

    subplot(num_files, 3, 3 * (file_idx - 1) + 2);
    builtin('plot', ...
        shape_data{file_idx}.t, ...
        shape_data{file_idx}.K, ...
        'LineWidth', 1.0, ...
        'Color', colors(file_idx, :));
    grid on;
    if file_idx == 1
        title("K");
    end
    if file_idx == num_files
        xlabel("Time [s]");
    end

    subplot(num_files, 3, 3 * (file_idx - 1) + 3);
    builtin('plot', ...
        shape_data{file_idx}.t, ...
        shape_data{file_idx}.damping, ...
        'LineWidth', 1.0, ...
        'Color', colors(file_idx, :));
    grid on;
    if file_idx == 1
        title("Damping");
    end
    if file_idx == num_files
        xlabel("Time [s]");
    end
end


function data = read_shape_dataset(read_csv_file, plot_label, ...
                                   use_transform, offset_x, ...
                                   offset_y, rotation_deg)
    %READ_SHAPE_DATASET Read one Schunk CSV and compute metrics.
    %
    % Inputs:
    %   read_csv_file - CSV file path
    %   plot_label - name used in plots and printed metrics
    %   use_transform - true to offset/rotate X,Y before plotting
    %   offset_x - robot x offset [m]
    %   offset_y - robot y offset [m]
    %   rotation_deg - robot planar rotation [deg]
    %
    % Outputs:
    %   data - struct with trajectory, controller signals, and metrics

    schunk_table = readtable(read_csv_file);

    t = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
    x = schunk_table.X;
    y = schunk_table.Y;

    if use_transform
        [x, y] = transform_robot_xy(x, y, offset_x, offset_y, ...
                                    rotation_deg);
    end

    speed_xy = read_speed_xy(schunk_table, t, x, y);
    mean_force = compute_mean_force(schunk_table);
    [dsj, path_length, movement_time] = compute_dsj(t, x, y);

    data.label = plot_label;
    data.file = read_csv_file;
    data.t = t;
    data.x = x;
    data.y = y;
    data.speed_xy = speed_xy;
    data.dsj = dsj;
    data.path_length = path_length;
    data.movement_time = movement_time;
    data.mean_force = mean_force;
    data.beta = read_beta_signal(schunk_table);
    data.K = read_K_signal(schunk_table);
    data.damping = read_damping_signal(schunk_table);
end


function [x_tf, y_tf] = transform_robot_xy(x, y, offset_x, offset_y, ...
                                           rotation_deg)
    %TRANSFORM_ROBOT_XY Remove offset and rotate the robot XY path.
    x = x - offset_x;
    y = y - offset_y;

    theta = deg2rad(rotation_deg);

    x_tf = cos(theta) * x - sin(theta) * y;
    y_tf = sin(theta) * x + cos(theta) * y;
end


function speed_xy = read_speed_xy(schunk_table, t, x, y)
    %READ_SPEED_XY Read logged planar speed or compute it from X,Y.
    if has_column(schunk_table, "v_meas1") && ...
            has_column(schunk_table, "v_meas2")
        speed_xy = sqrt(schunk_table.v_meas1.^2 + ...
                        schunk_table.v_meas2.^2);
    else
        vx = gradient(x, t);
        vy = gradient(y, t);
        speed_xy = sqrt(vx.^2 + vy.^2);
    end
end


function mean_force = compute_mean_force(schunk_table)
    %COMPUTE_MEAN_FORCE Mean planar force magnitude from FT1 and FT2.
    force_xy = sqrt(schunk_table.FT1.^2 + schunk_table.FT2.^2);
    mean_force = mean(force_xy);
end


function [dsj, path_length, movement_time] = compute_dsj(t, x, y)
    %COMPUTE_DSJ Chen paper dimensionless squared jerk metric.
    %
    % Formula:
    %   DSJ = integral(jerk_x^2 + jerk_y^2) dt * T^5 / A^2
    % where T is movement time [s], and A is planar path length [m].

    movement_time = t(end) - t(1);
    step_length = sqrt(diff(x).^2 + diff(y).^2);
    path_length = sum(step_length);

    vx_dsj = gradient(x, t);
    vy_dsj = gradient(y, t);
    ax_dsj = gradient(vx_dsj, t);
    ay_dsj = gradient(vy_dsj, t);
    jx_dsj = gradient(ax_dsj, t);
    jy_dsj = gradient(ay_dsj, t);

    jerk_squared = jx_dsj.^2 + jy_dsj.^2;
    dsj = trapz(t, jerk_squared) * movement_time^5 / path_length^2;
end


function beta = read_beta_signal(schunk_table)
    %READ_BETA_SIGNAL Read beta from Chen or Kang logs.
    if has_column(schunk_table, "beta_hat")
        beta = schunk_table.beta_hat;
    elseif has_column(schunk_table, "beta")
        beta = schunk_table.beta;
    else
        beta = nan(height(schunk_table), 1);
    end
end


function K = read_K_signal(schunk_table)
    %READ_K_SIGNAL Read K from Chen logs if present.
    if has_column(schunk_table, "K_hat")
        K = schunk_table.K_hat;
    else
        K = nan(height(schunk_table), 1);
    end
end


function damping = read_damping_signal(schunk_table)
    %READ_DAMPING_SIGNAL Read one scalar damping trace from each log type.
    if has_column(schunk_table, "bd_applied")
        damping = schunk_table.bd_applied;
    elseif has_column(schunk_table, "Ci11") && ...
            has_column(schunk_table, "Ci22") && ...
            has_column(schunk_table, "Ci33")
        damping = (schunk_table.Ci11 + schunk_table.Ci22 + ...
                   schunk_table.Ci33) / 3.0;
    else
        damping = nan(height(schunk_table), 1);
    end
end


function result = has_column(schunk_table, column_name)
    %HAS_COLUMN Return true when table has a named column.
    result = any(strcmp(schunk_table.Properties.VariableNames, ...
                        column_name));
end
