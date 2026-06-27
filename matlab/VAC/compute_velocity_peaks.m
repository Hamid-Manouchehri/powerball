% Compute XY speed profiles and detect speed peaks for VAC robot trials.
%
% This script reads each robot shape/controller CSV file, computes planar
% end-effector speed, detects local speed peaks, and writes processed CSV
% files for later plotting and comparison.
%
% XY speed:
%   speed_xy = sqrt(v_meas1^2 + v_meas2^2)
%
% Inputs:
%   read_raw_data_dir: folder with robot CSV logs
%   read_free_hand_data_dir: folder with free-hand CSV logs
%
% Outputs:
%   write_velocity_profile_csv_file: one row per time sample
%   write_velocity_peaks_csv_file: one row per detected speed peak

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_raw_data_dir = "./raw_data/";  % TODO robot raw data folder
read_free_hand_data_dir = "./raw_data/free_hand/";  % TODO free-hand folder
write_velocity_profile_csv_file = ...  % TODO output speed profile CSV
    "./processed_data/velocity_profiles.csv";
write_velocity_peaks_csv_file = ...  % TODO output speed peak CSV
    "./processed_data/velocity_peaks.csv";

subject_id = 1;  % TODO subject id in output CSV

shape_ids = [1, 2, 3, 4];  % TODO [eight, ellipse, clover, squircle]
shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

controller_ids = [1, 2];  % TODO [chen, kang_indirect]
controller_names = ["Chen", "Kang"];  % TODO print labels
controller_file_names = [
    "chen_var_adm_schunk"           % TODO Chen filename suffix
    "kang_indirect_var_adm_schunk"  % TODO Kang filename suffix
];

save_speed_profile_table = true;  % TODO true or false
save_peak_table = true;           % TODO true or false

smoothing_window_samples = 5;      % TODO samples for speed smoothing
min_peak_speed = 0.02;             % TODO minimum peak speed [m/s]
min_peak_distance_samples = 50;    % TODO minimum peak spacing [samples]
free_hand_subject_id = 1;          % TODO free-hand subject id
free_hand_controller_id = 0;       % TODO free-hand controller id
free_hand_shape_ids = [1, 2, 3, 4];  % TODO same order as file names
free_hand_shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

% -------------------- Prepare Output Columns --------------------
profile_subject_id = [];
profile_shape_id = [];
profile_controller_id = [];
profile_sample = [];
profile_time_s = [];
profile_x = [];
profile_y = [];
profile_speed_xy = [];
profile_speed_xy_smooth = [];
profile_source_file = strings(0, 1);
profile_data_source = strings(0, 1);

peak_subject_id = [];
peak_shape_id = [];
peak_controller_id = [];
peak_id = [];
peak_sample = [];
peak_time_s = [];
peak_x = [];
peak_y = [];
peak_speed_xy = [];
peak_speed_xy_smooth = [];
peak_source_file = strings(0, 1);
peak_data_source = strings(0, 1);

% -------------------- Compute Speed Profiles And Peaks --------------------
fprintf("\nVelocity peak detection\n");

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_ids)
        read_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
            "_" + controller_file_names(controller_idx) + ".csv";

        raw_table = readtable(read_csv_file);

        t = (raw_table.Time_us - raw_table.Time_us(1)) / 1e6;
        x = raw_table.X;
        y = raw_table.Y;
        speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);
        speed_xy_smooth = movmean(speed_xy, smoothing_window_samples);

        sample = (1:height(raw_table))';

        profile_subject_id = [profile_subject_id; ...
            subject_id * ones(height(raw_table), 1)];
        profile_shape_id = [profile_shape_id; ...
            shape_ids(shape_idx) * ones(height(raw_table), 1)];
        profile_controller_id = [profile_controller_id; ...
            controller_ids(controller_idx) * ones(height(raw_table), 1)];
        profile_sample = [profile_sample; sample];
        profile_time_s = [profile_time_s; t];
        profile_x = [profile_x; x];
        profile_y = [profile_y; y];
        profile_speed_xy = [profile_speed_xy; speed_xy];
        profile_speed_xy_smooth = [profile_speed_xy_smooth; ...
            speed_xy_smooth];
        profile_source_file = [profile_source_file; ...
            repmat(read_csv_file, height(raw_table), 1)];

        candidate_peak_idx = find_candidate_speed_peaks( ...
            speed_xy_smooth, min_peak_speed);
        selected_peak_idx = select_peaks_by_distance( ...
            candidate_peak_idx, speed_xy_smooth, min_peak_distance_samples);

        for peak_idx = 1:numel(selected_peak_idx)
            sample_idx = selected_peak_idx(peak_idx);

            peak_subject_id(end + 1, 1) = subject_id;
            peak_shape_id(end + 1, 1) = shape_ids(shape_idx);
            peak_controller_id(end + 1, 1) = controller_ids(controller_idx);
            peak_id(end + 1, 1) = peak_idx;
            peak_sample(end + 1, 1) = sample_idx;
            peak_time_s(end + 1, 1) = t(sample_idx);
            peak_x(end + 1, 1) = x(sample_idx);
            peak_y(end + 1, 1) = y(sample_idx);
            peak_speed_xy(end + 1, 1) = speed_xy(sample_idx);
            peak_speed_xy_smooth(end + 1, 1) = ...
                speed_xy_smooth(sample_idx);
            peak_source_file(end + 1, 1) = read_csv_file;
        end

        fprintf("%s, %s: %d peaks\n", ...
            shape_names(shape_idx), controller_names(controller_idx), ...
            numel(selected_peak_idx));
    end
end

for shape_idx = 1:numel(free_hand_shape_names)
    read_csv_file = read_free_hand_data_dir + ...
        free_hand_shape_names(shape_idx) + ".csv";

    raw_table = readtable(read_csv_file);

    x = raw_table.x;
    y = raw_table.y;
    t = raw_table.time_s;

    valid_sample_idx = isfinite(x) & isfinite(y) & isfinite(t);
    x = x(valid_sample_idx);
    y = y(valid_sample_idx);
    t = t(valid_sample_idx);
    t = t - t(1);

    vx = gradient(x, t);
    vy = gradient(y, t);

    speed_xy = sqrt(vx.^2 + vy.^2);
    speed_xy_smooth = movmean(speed_xy, smoothing_window_samples);

    sample = (1:numel(t))';

    profile_subject_id = [profile_subject_id; ...
        free_hand_subject_id * ones(numel(t), 1)];
    profile_shape_id = [profile_shape_id; free_hand_shape_ids(shape_idx) * ...
        ones(numel(t), 1)];
    profile_controller_id = [profile_controller_id; ...
        free_hand_controller_id * ones(numel(t), 1)];
    profile_sample = [profile_sample; sample];
    profile_time_s = [profile_time_s; t];
    profile_x = [profile_x; x];
    profile_y = [profile_y; y];
    profile_speed_xy = [profile_speed_xy; speed_xy];
    profile_speed_xy_smooth = [profile_speed_xy_smooth; speed_xy_smooth];
    profile_source_file = [profile_source_file; ...
        repmat(read_csv_file, numel(t), 1)];
    profile_data_source = [profile_data_source; ...
        repmat("free_hand", numel(t), 1)];

    candidate_peak_idx = find_candidate_speed_peaks( ...
        speed_xy_smooth, min_peak_speed);
    selected_peak_idx = select_peaks_by_distance( ...
        candidate_peak_idx, speed_xy_smooth, min_peak_distance_samples);

    for peak_idx = 1:numel(selected_peak_idx)
        sample_idx = selected_peak_idx(peak_idx);

        peak_subject_id(end + 1, 1) = free_hand_subject_id;
        peak_shape_id(end + 1, 1) = free_hand_shape_ids(shape_idx);
        peak_controller_id(end + 1, 1) = free_hand_controller_id;
        peak_id(end + 1, 1) = peak_idx;
        peak_sample(end + 1, 1) = sample_idx;
        peak_time_s(end + 1, 1) = t(sample_idx);
        peak_x(end + 1, 1) = x(sample_idx);
        peak_y(end + 1, 1) = y(sample_idx);
        peak_speed_xy(end + 1, 1) = speed_xy(sample_idx);
        peak_speed_xy_smooth(end + 1, 1) = speed_xy_smooth(sample_idx);
        peak_source_file(end + 1, 1) = read_csv_file;
        peak_data_source(end + 1, 1) = "free_hand";
    end

    fprintf("free_hand, %s: %d peaks\n", ...
        free_hand_shape_names(shape_idx), numel(selected_peak_idx));
end

% -------------------- Write Speed Profile CSV --------------------
if save_speed_profile_table
    velocity_profile_table = table( ...
        profile_subject_id, profile_shape_id, profile_controller_id, ...
        profile_sample, profile_time_s, profile_x, profile_y, ...
        profile_speed_xy, profile_speed_xy_smooth, profile_source_file, ...
        profile_data_source, 'VariableNames', {'subject_id', ...
        'shape_id', 'controller_id', 'sample', 'time_s', 'x', 'y', ...
        'speed_xy', 'speed_xy_smooth', 'source_file', 'data_source'});

    writetable(velocity_profile_table, write_velocity_profile_csv_file);

    fprintf("\nVelocity profile table written to:\n");
    fprintf("%s\n", write_velocity_profile_csv_file);
    fprintf("Total profile rows = %d\n", height(velocity_profile_table));
end

% -------------------- Write Speed Peak CSV --------------------
if save_peak_table
    velocity_peak_table = table( ...
        peak_subject_id, peak_shape_id, peak_controller_id, peak_id, ...
        peak_sample, peak_time_s, peak_x, peak_y, peak_speed_xy, ...
        peak_speed_xy_smooth, peak_source_file, peak_data_source, ...
        'VariableNames', ...
        {'subject_id', 'shape_id', 'controller_id', 'peak_id', ...
        'peak_sample', 'peak_time_s', 'peak_x', 'peak_y', ...
        'peak_speed_xy', 'peak_speed_xy_smooth', 'source_file', ...
        'data_source'});

    writetable(velocity_peak_table, write_velocity_peaks_csv_file);

    fprintf("\nVelocity peak table written to:\n");
    fprintf("%s\n", write_velocity_peaks_csv_file);
    fprintf("Total peaks = %d\n", height(velocity_peak_table));
end


function candidate_peak_idx = find_candidate_speed_peaks(speed, ...
                                                         min_peak_speed)
    %FIND_CANDIDATE_SPEED_PEAKS Find local maxima above a speed threshold.

    candidate_peak_idx = [];

    for sample_idx = 2:(numel(speed) - 1)
        is_local_peak = speed(sample_idx) > speed(sample_idx - 1) & ...
                        speed(sample_idx) >= speed(sample_idx + 1);
        is_fast_enough = speed(sample_idx) >= min_peak_speed;

        if is_local_peak && is_fast_enough
            candidate_peak_idx(end + 1, 1) = sample_idx;
        end
    end
end


function selected_peak_idx = select_peaks_by_distance( ...
    candidate_peak_idx, speed, min_peak_distance_samples)
    %SELECT_PEAKS_BY_DISTANCE Keep strong peaks separated by sample distance.

    selected_peak_idx = [];

    [~, sort_idx] = sort(speed(candidate_peak_idx), "descend");
    sorted_peak_idx = candidate_peak_idx(sort_idx);

    for peak_idx = 1:numel(sorted_peak_idx)
        sample_idx = sorted_peak_idx(peak_idx);

        if isempty(selected_peak_idx)
            selected_peak_idx(end + 1, 1) = sample_idx;
        else
            sample_distance = abs(selected_peak_idx - sample_idx);
            is_far_enough = all(sample_distance >= min_peak_distance_samples);

            if is_far_enough
                selected_peak_idx(end + 1, 1) = sample_idx;
            end
        end
    end

    selected_peak_idx = sort(selected_peak_idx);
end
