% Compute windowed DSJ profiles from VAC robot raw datasets.
%
% This script reads each robot shape/controller CSV file, computes
% dimensionless squared jerk (DSJ) over fixed-size sample windows, and
% writes all window values to one profile CSV file.
%
% Windowed DSJ:
%   DSJ = integral(jerk^2) * movement_time^5 / path_length^2
%
% Inputs:
%   read_raw_data_dir: folder with robot CSV logs
%
% Outputs:
%   write_window_metrics_csv_file: one row per windowed metric value

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_raw_data_dir = "./raw_data/";  % TODO robot raw data folder
write_window_metrics_csv_file = ...  % TODO output profile CSV
    "./processed_data/window_metrics.csv";

subject_id = 1;  % TODO subject id in output CSV

shape_ids = [1, 2, 3, 4];  % TODO [eight, ellipse, clover, squircle]
shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

controller_ids = [1, 2];  % TODO [chen, kang_indirect]
controller_names = ["Chen", "Kang"];  % TODO plot labels
controller_file_names = [
    "chen_var_adm_schunk"           % TODO Chen filename suffix
    "kang_indirect_var_adm_schunk"  % TODO Kang filename suffix
];

window_size_samples = 300;  % TODO samples per DSJ window
window_step_samples = 30;   % TODO samples between window starts
minimum_path_length = 0.005;  % TODO minimum path length [m]

% -------------------- Prepare Output Columns --------------------
out_subject_id = [];
out_shape_id = [];
out_controller_id = [];
out_metric_name = strings(0, 1);
out_window_id = [];
out_start_sample = [];
out_end_sample = [];
out_start_time_s = [];
out_end_time_s = [];
out_center_time_s = [];
out_window_size_samples = [];
out_window_step_samples = [];
out_window_duration_s = [];
out_path_length = [];
out_value = [];

% -------------------- Compute Windowed DSJ --------------------
fprintf("\nWindowed DSJ estimates\n");

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_ids)
        read_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
            "_" + controller_file_names(controller_idx) + ".csv";

        raw_table = readtable(read_csv_file);

        t = (raw_table.Time_us - raw_table.Time_us(1)) / 1e6;
        x = raw_table.X;
        y = raw_table.Y;
        vx = raw_table.v_meas1;
        vy = raw_table.v_meas2;

        num_samples = height(raw_table);
        window_id = 0;

        for start_sample = 1:window_step_samples:num_samples
            end_sample = start_sample + window_size_samples - 1;

            if end_sample > num_samples
                break;
            end

            window_id = window_id + 1;
            sample_idx = start_sample:end_sample;

            t_window = t(sample_idx);
            x_window = x(sample_idx);
            y_window = y(sample_idx);
            vx_window = vx(sample_idx);
            vy_window = vy(sample_idx);

            [DSJ_window, path_length, window_duration] = ...
                compute_window_DSJ( ...
                    t_window, x_window, y_window, ...
                    vx_window, vy_window, minimum_path_length);

            out_subject_id(end + 1, 1) = subject_id;
            out_shape_id(end + 1, 1) = shape_ids(shape_idx);
            out_controller_id(end + 1, 1) = controller_ids(controller_idx);
            out_metric_name(end + 1, 1) = "DSJ";
            out_window_id(end + 1, 1) = window_id;
            out_start_sample(end + 1, 1) = start_sample;
            out_end_sample(end + 1, 1) = end_sample;
            out_start_time_s(end + 1, 1) = t_window(1);
            out_end_time_s(end + 1, 1) = t_window(end);
            out_center_time_s(end + 1, 1) = ...
                0.5 * (t_window(1) + t_window(end));
            out_window_size_samples(end + 1, 1) = window_size_samples;
            out_window_step_samples(end + 1, 1) = window_step_samples;
            out_window_duration_s(end + 1, 1) = window_duration;
            out_path_length(end + 1, 1) = path_length;
            out_value(end + 1, 1) = DSJ_window;
        end

        fprintf("%s, %s: %d windows\n", ...
            shape_names(shape_idx), controller_names(controller_idx), ...
            window_id);
    end
end

% -------------------- Write Output CSV --------------------
window_metrics_table = table( ...
    out_subject_id, out_shape_id, out_controller_id, out_metric_name, ...
    out_window_id, out_start_sample, out_end_sample, ...
    out_start_time_s, out_end_time_s, out_center_time_s, ...
    out_window_size_samples, out_window_step_samples, ...
    out_window_duration_s, out_path_length, out_value, ...
    'VariableNames', {'subject_id', 'shape_id', 'controller_id', ...
    'metric_name', 'window_id', 'start_sample', 'end_sample', ...
    'start_time_s', 'end_time_s', 'center_time_s', ...
    'window_size_samples', 'window_step_samples', ...
    'window_duration_s', 'path_length', 'value'});

writetable(window_metrics_table, write_window_metrics_csv_file);

fprintf("\nWindowed DSJ table written to:\n");
fprintf("%s\n", write_window_metrics_csv_file);
fprintf("Total rows = %d\n", height(window_metrics_table));


function [DSJ_window, path_length, window_duration] = compute_window_DSJ( ...
    t_window, x_window, y_window, vx_window, vy_window, minimum_path_length)
    %COMPUTE_WINDOW_DSJ Compute DSJ inside one time window.
    %
    % Inputs:
    %   t_window - time samples [s]
    %   x_window, y_window - end-effector position [m]
    %   vx_window, vy_window - end-effector velocity [m/s]
    %   minimum_path_length - minimum useful motion distance [m]
    %
    % Outputs:
    %   DSJ_window - windowed dimensionless squared jerk [-]
    %   path_length - planar path length inside the window [m]
    %   window_duration - window movement time [s]

    window_duration = t_window(end) - t_window(1);

    step_length = sqrt(diff(x_window).^2 + diff(y_window).^2);
    path_length = sum(step_length);

    DSJ_window = NaN;

    if path_length < minimum_path_length
        return;
    end

    ax_window = gradient(vx_window, t_window);
    ay_window = gradient(vy_window, t_window);
    jx_window = gradient(ax_window, t_window);
    jy_window = gradient(ay_window, t_window);

    jerk_squared = jx_window.^2 + jy_window.^2;

    DSJ_window = trapz(t_window, jerk_squared) * ...
        window_duration^5 / path_length^2;
end
