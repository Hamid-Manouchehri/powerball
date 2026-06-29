% Compute whole-shape power-law parameters for one free-hand drawing.
%
% This script reads one timestamped free-hand drawing CSV file, estimates
% one velocity-curvature power law over the complete trajectory, and writes
% only the matching row in the free-hand power-law CSV. Existing rows and
% columns are preserved.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   read_csv_file: free-hand CSV file with x, y, and time_s columns
%   subject_id: subject identifier used in output CSV
%   shape_id: shape identifier used in output CSV
%   controller_id: controller identifier used in output CSV
%
% Outputs:
%   write_powerlaw_csv_file: updated with beta, K, R2, and RMSE columns

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_csv_file = "./raw_data/free_hand/clover.csv";  % TODO input CSV

subject_id = 1;       % TODO [sub1:1]
shape_id = 3;         % TODO [eight:1, ellipse:2, four_leaves:3, squircle:4]
shape_name = "clover";  % TODO shape label
controller_id = 0;    % TODO [free_hand:0]

write_powerlaw_csv_file = ...
    "./processed_data/free_hand_powerlaw.csv";  % TODO output CSV

min_speed = 0.001;             % TODO minimum speed [data unit/s]
min_curvature = 0.001;         % TODO minimum curvature [1/data unit]
max_curvature = 200.0;         % TODO maximum curvature [1/data unit]
minimum_fit_points = 10;       % TODO minimum samples for LS fit

beta_min = 0.0;                % TODO lower beta bound
beta_max = 2.0 / 3.0;          % TODO upper beta bound

% -------------------- Read Free-Hand Trial --------------------
raw_table = readtable(read_csv_file);

x = raw_table.x;
y = raw_table.y;
time_s = raw_table.time_s;

valid_sample_idx = isfinite(x) & isfinite(y) & isfinite(time_s);
x = x(valid_sample_idx);
y = y(valid_sample_idx);
time_s = time_s(valid_sample_idx);
time_s = time_s - time_s(1);

% -------------------- Estimate Whole-Shape Power Law --------------------
[beta, K, R2, RMSE, valid_count] = estimate_free_hand_power_law( ...
    x, y, time_s, min_speed, min_curvature, max_curvature, ...
    minimum_fit_points, beta_min, beta_max);

duration_s = time_s(end) - time_s(1);

% -------------------- Write/Update Free-Hand CSV --------------------
if isfile(write_powerlaw_csv_file)
    powerlaw_table = readtable(write_powerlaw_csv_file);
else
    powerlaw_table = table();
end

powerlaw_table = update_powerlaw_table(powerlaw_table, subject_id, ...
    shape_id, shape_name, controller_id, beta, K, R2, RMSE, ...
    valid_count, duration_s);

writetable(powerlaw_table, write_powerlaw_csv_file);

fprintf("\nFree-hand power-law values updated in:\n");
fprintf("%s\n", write_powerlaw_csv_file);
fprintf("subject_id = %d, shape_id = %d, controller_id = %d\n", ...
    subject_id, shape_id, controller_id);
fprintf("shape_name = %s\n", shape_name);
fprintf("beta = %.10f\n", beta);
fprintf("K = %.10f\n", K);
fprintf("R2 = %.10f\n", R2);
fprintf("RMSE = %.10f\n", RMSE);
fprintf("valid_count = %d\n", valid_count);
fprintf("duration_s = %.10f\n", duration_s);
fprintf("Rows preserved = %d\n", height(powerlaw_table));


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


function powerlaw_table = update_powerlaw_table( ...
    powerlaw_table, subject_id, shape_id, shape_name, controller_id, ...
    beta, K, R2, RMSE, valid_count, duration_s)
    %UPDATE_POWERLAW_TABLE Upsert one free-hand power-law result row.

    if height(powerlaw_table) == 0
        powerlaw_table = table(subject_id, shape_id, shape_name, ...
            controller_id, beta, K, R2, RMSE, valid_count, duration_s, ...
            'VariableNames', {'subject_id', 'shape_id', 'shape_name', ...
            'controller_id', 'beta', 'K', 'R2', 'RMSE', ...
            'valid_count', 'duration_s'});
        return;
    end

    powerlaw_table = add_missing_columns(powerlaw_table);

    row_idx = powerlaw_table.subject_id == subject_id & ...
              powerlaw_table.shape_id == shape_id & ...
              powerlaw_table.controller_id == controller_id;

    if any(row_idx)
        powerlaw_table.shape_name(row_idx) = shape_name;
        powerlaw_table.beta(row_idx) = beta;
        powerlaw_table.K(row_idx) = K;
        powerlaw_table.R2(row_idx) = R2;
        powerlaw_table.RMSE(row_idx) = RMSE;
        powerlaw_table.valid_count(row_idx) = valid_count;
        powerlaw_table.duration_s(row_idx) = duration_s;
    else
        new_row = make_appended_row(powerlaw_table, subject_id, ...
            shape_id, shape_name, controller_id, beta, K, R2, RMSE, ...
            valid_count, duration_s);
        powerlaw_table = [powerlaw_table; new_row];
    end
end


function powerlaw_table = add_missing_columns(powerlaw_table)
    %ADD_MISSING_COLUMNS Add expected columns without changing old data.

    if ~ismember("subject_id", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.subject_id = nan(height(powerlaw_table), 1);
    end

    if ~ismember("shape_name", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.shape_name = strings(height(powerlaw_table), 1);
    else
        powerlaw_table.shape_name = string(powerlaw_table.shape_name);
    end

    if ~ismember("controller_id", ...
                 string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.controller_id = nan(height(powerlaw_table), 1);
    end

    if ~ismember("beta", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.beta = nan(height(powerlaw_table), 1);
    end

    if ~ismember("K", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.K = nan(height(powerlaw_table), 1);
    end

    if ~ismember("R2", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.R2 = nan(height(powerlaw_table), 1);
    end

    if ~ismember("RMSE", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.RMSE = nan(height(powerlaw_table), 1);
    end

    if ~ismember("valid_count", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.valid_count = nan(height(powerlaw_table), 1);
    end

    if ~ismember("duration_s", string(powerlaw_table.Properties.VariableNames))
        powerlaw_table.duration_s = nan(height(powerlaw_table), 1);
    end
end


function new_row = make_appended_row( ...
    powerlaw_table, subject_id, shape_id, shape_name, controller_id, ...
    beta, K, R2, RMSE, valid_count, duration_s)
    %MAKE_APPENDED_ROW Create one row with the output table schema.

    new_row = powerlaw_table(1, :);

    for var_idx = 1:width(new_row)
        var_name = new_row.Properties.VariableNames{var_idx};
        new_row.(var_name) = get_missing_value(new_row.(var_name));
    end

    new_row.subject_id = subject_id;
    new_row.shape_id = shape_id;
    new_row.shape_name = shape_name;
    new_row.controller_id = controller_id;
    new_row.beta = beta;
    new_row.K = K;
    new_row.R2 = R2;
    new_row.RMSE = RMSE;
    new_row.valid_count = valid_count;
    new_row.duration_s = duration_s;
end


function value = get_missing_value(example_value)
    %GET_MISSING_VALUE Return a missing value that matches the input type.

    if isnumeric(example_value)
        value = NaN;
    elseif islogical(example_value)
        value = false;
    elseif isstring(example_value)
        value = missing;
    elseif ischar(example_value)
        value = '';
    elseif iscategorical(example_value)
        value = categorical(missing);
    elseif iscell(example_value)
        value = {missing};
    else
        value = missing;
    end
end
