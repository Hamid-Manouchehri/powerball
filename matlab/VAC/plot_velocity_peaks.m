% Plot XY speed profiles and highlight detected peaks for VAC trials.
%
% This script reads the processed velocity profile and peak tables, then
% plots speed magnitude over time in a fixed 2x4 layout. Chen is shown in
% the first row and Kang is shown in the second row.
%
% Inputs:
%   read_velocity_profile_csv_file: sample-by-sample speed table
%   read_velocity_peaks_csv_file: detected peak table
%
% Outputs:
%   Figure:
%   1) XY speed profile with peak markers for each subject/shape/controller

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_velocity_profile_csv_file = ...  % TODO profile CSV
    "./processed_data/velocity_profiles.csv";
read_velocity_peaks_csv_file = ...  % TODO peak CSV
    "./processed_data/velocity_peaks.csv";

shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO
shape_ids = [1, 2, 3, 4];  % TODO [eight, ellipse, clover, squircle]
controller_ids = [1, 2];  % TODO [chen, kang_indirect]
controller_names = ["Chen", "Kang"];  % TODO plot labels

speed_ylim = [0, NaN];  % TODO leave upper limit as NaN for auto-scale

% -------------------- Read Tables --------------------
velocity_profile_table = readtable(read_velocity_profile_csv_file);
velocity_peak_table = readtable(read_velocity_peaks_csv_file);
subject_id = velocity_profile_table.subject_id(1);

figure(Name="velocity_peaks_over_time", NumberTitle="off");
tiledlayout(2, numel(shape_names), "TileSpacing", "compact", ...
            "Padding", "compact");

for controller_idx = 1:numel(controller_names)
    for shape_idx = 1:numel(shape_names)
        shape_id = shape_ids(shape_idx);
        controller_id = controller_ids(controller_idx);

        profile_row_idx = velocity_profile_table.subject_id == subject_id & ...
                          velocity_profile_table.shape_id == shape_id & ...
                          velocity_profile_table.controller_id == ...
                          controller_id;

        peak_row_idx = velocity_peak_table.subject_id == subject_id & ...
                       velocity_peak_table.shape_id == shape_id & ...
                       velocity_peak_table.controller_id == controller_id;

        case_profile = velocity_profile_table(profile_row_idx, :);
        case_peaks = velocity_peak_table(peak_row_idx, :);

        tile_idx = (controller_idx - 1) * numel(shape_names) + shape_idx;
        nexttile(tile_idx);
        hold on;
        grid on;

        plot(case_profile.time_s, case_profile.speed_xy, ...
            'Color', [0.75 0.75 0.75], 'LineWidth', 0.8);
        plot(case_profile.time_s, case_profile.speed_xy_smooth, ...
            'Color', [0.0 0.45 0.74], 'LineWidth', 1.2);

        if ~isempty(case_peaks)
            scatter(case_peaks.peak_time_s, ...
                case_peaks.peak_speed_xy_smooth, 34, 'filled', ...
                'MarkerFaceColor', [0.85 0.1 0.1], ...
                'MarkerEdgeColor', [0.85 0.1 0.1]);
        end

        if controller_idx == 1
            title(shape_names(shape_idx));
        end

        if shape_idx == 1
            ylabel(controller_names(controller_idx) + ...
                newline + "Speed [m/s]");
        else
            ylabel("Speed [m/s]");
        end

        xlabel("Time [s]");

        if ~isnan(speed_ylim(2))
            ylim(speed_ylim);
        end
    end
end

sgtitle("XY Speed Profiles with Detected Peaks");
