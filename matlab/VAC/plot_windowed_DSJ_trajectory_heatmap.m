% Plot windowed DSJ values over robot XY trajectories.
%
% This script reads robot raw trajectory files and the windowed metric table
% from compute_windowed_DSJ_.m. Each trajectory segment is colored by its
% windowed DSJ value.
%
% Inputs:
%   read_raw_data_dir: folder with robot CSV logs
%   read_window_metrics_csv_file: windowed metric table
%
% Outputs:
%   Figure:
%   1) XY trajectory colored by windowed DSJ for each shape/controller case

clc;
clear;

% -------------------- User Settings --------------------
read_raw_data_dir = "./raw_data/";  % TODO robot raw data folder
read_window_metrics_csv_file = ...  % TODO windowed metric CSV
    "./processed_data/window_metrics.csv";

subject_id = 1;  % TODO subject id in window metric CSV

shape_ids = [1, 2, 3, 4];  % TODO [eight, ellipse, clover, squircle]
shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

controller_ids = [1, 2];  % TODO [chen, kang_indirect]
controller_names = ["Chen", "Kang"];  % TODO plot labels
controller_file_names = [
    "chen_var_adm_schunk"           % TODO Chen filename suffix
    "kang_indirect_var_adm_schunk"  % TODO Kang filename suffix
];

DSJ_scale = 1e9;  % TODO divide DSJ values for easier colorbar numbers
marker_size = 8;  % TODO scatter marker size

% -------------------- Read Windowed DSJ Table --------------------
window_metrics = readtable(read_window_metrics_csv_file);
metric_name = string(window_metrics.metric_name);
window_metrics = window_metrics(metric_name == "DSJ", :);

valid_DSJ_idx = isfinite(window_metrics.value);
DSJ_min = min(window_metrics.value(valid_DSJ_idx)) / DSJ_scale;
DSJ_max = max(window_metrics.value(valid_DSJ_idx)) / DSJ_scale;

% -------------------- Plot Windowed DSJ Over XY --------------------
figure(Name="windowed_DSJ_over_xy_trajectory", NumberTitle="off");
tiledlayout(numel(controller_names), numel(shape_names), ...
            "TileSpacing", "compact", "Padding", "compact");

for controller_idx = 1:numel(controller_names)
    for shape_idx = 1:numel(shape_names)
        read_raw_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
            "_" + controller_file_names(controller_idx) + ".csv";

        raw_table = readtable(read_raw_csv_file);

        row_idx = window_metrics.subject_id == subject_id & ...
                  window_metrics.shape_id == shape_ids(shape_idx) & ...
                  window_metrics.controller_id == ...
                  controller_ids(controller_idx);

        case_windows = window_metrics(row_idx, :);

        nexttile;
        hold on;
        grid on;
        axis equal;

        for window_idx = 1:height(case_windows)
            start_sample = case_windows.start_sample(window_idx);
            end_sample = case_windows.end_sample(window_idx);
            sample_idx = start_sample:end_sample;

            DSJ_value = case_windows.value(window_idx) / DSJ_scale;
            DSJ_color = DSJ_value * ones(numel(sample_idx), 1);

            scatter(raw_table.X(sample_idx), raw_table.Y(sample_idx), ...
                marker_size, DSJ_color, 'filled');
        end

        clim([DSJ_min, DSJ_max]);

        title(shape_names(shape_idx) + " " + controller_names(controller_idx));
        xlabel("X [m]");
        ylabel("Y [m]");
    end
end

colormap turbo;
colorbar;
