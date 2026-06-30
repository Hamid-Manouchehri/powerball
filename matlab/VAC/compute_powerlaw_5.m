% Compute whole-shape power-law parameters for one VAC raw dataset.
%
% This script reads one subject/shape/controller trial, estimates one
% velocity-curvature power law over the complete trajectory, and writes
% only the matching row in the master CSV. Existing rows and columns are
% preserved.
%
% Power law:
%   v = K * kappa^(-beta)
%
% Least-squares log form:
%   log(v) = log(K) - beta * log(kappa)
%
% Inputs:
%   read_csv_file: raw trial CSV file
%   subject_id: subject identifier used in master CSV
%   shape_id: shape identifier used in master CSV
%   controller_id: controller identifier used in master CSV
%
% Outputs:
%   write_master_csv_file: updated with beta, K, R2, and RMSE columns

clc;
clear;
close all;

% -------------------- User Settings --------------------
% read_csv_file = "./raw_data/eight_chen_var_adm_schunk.csv";  % TODO
% read_csv_file = "./raw_data/nu_2_chen_var_adm_schunk.csv";  % TODO
% read_csv_file = "./raw_data/nu_2_chen_kf_var_adm_schunk.csv";  % TODO

subject_id = 1;       % TODO [sub1:1]
shape_id = 2;         % TODO [eight:1, ellipse:2, four_leaves:3, squircle:4]
controller_id = 4;    % TODO [chen:1, kang_indirect:2, chen_ls:3, chen_kf:4]

write_master_csv_file = ...
    "./processed_data/master_dataset_chen_ls_kf_study_2.csv";  % TODO

controller_dt = 0.005;          % TODO controller period [s]
min_speed = 0.005;              % TODO minimum speed [m/s]
min_curvature = 0.001;          % TODO minimum curvature [1/m]
max_curvature = 200.0;          % TODO maximum curvature [1/m]
minimum_fit_points = 10;        % TODO minimum samples for LS fit

beta_min = 0.0;                 % TODO lower beta bound
beta_max = 2.0 / 3.0;           % TODO upper beta bound

% -------------------- Read Raw Trial --------------------
raw_table = readtable(read_csv_file);

x = raw_table.X;
y = raw_table.Y;
xd = raw_table.v_meas1;
yd = raw_table.v_meas2;

% -------------------- Estimate Whole-Shape Power Law --------------------
[beta, K, R2, RMSE, valid_count] = estimate_whole_shape_power_law( ...
    x, y, xd, yd, controller_dt, min_speed, min_curvature, ...
    max_curvature, minimum_fit_points, beta_min, beta_max);

% -------------------- Write/Update Master CSV --------------------
if isfile(write_master_csv_file)
    master_table = readtable(write_master_csv_file);
else
    master_table = table();
end

master_table = update_master_table(master_table, subject_id, shape_id, ...
    controller_id, beta, K, R2, RMSE);

writetable(master_table, write_master_csv_file);

fprintf("\nPower-law values updated in:\n");
fprintf("%s\n", write_master_csv_file);
fprintf("subject_id = %d, shape_id = %d, controller_id = %d\n", ...
    subject_id, shape_id, controller_id);
fprintf("beta = %.10f\n", beta);
fprintf("K = %.10f\n", K);
fprintf("R2 = %.10f\n", R2);
fprintf("RMSE = %.10f\n", RMSE);
fprintf("valid_count = %d\n", valid_count);
fprintf("Rows preserved = %d\n", height(master_table));


function [beta, K, R2, RMSE, valid_count] = ...
    estimate_whole_shape_power_law( ...
        x, y, xd, yd, dt, min_speed, min_curvature, max_curvature, ...
        minimum_fit_points, beta_min, beta_max)
    %ESTIMATE_WHOLE_SHAPE_POWER_LAW Fit v = K*kappa^(-beta).
    %
    % Inputs:
    %   x, y - end-effector position [m]
    %   xd, yd - end-effector velocity [m/s]
    %   dt - sampling period [s]
    %
    % Outputs:
    %   beta - fitted power-law exponent [-]
    %   K - fitted velocity coefficient
    %   R2 - coefficient of determination of log-space fit [-]
    %   RMSE - root mean square error of log-space fit
    %   valid_count - number of samples used in the fit

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

    path_step = sqrt(diff(x).^2 + diff(y).^2);
    path_length = sum(path_step);
    fprintf("path length = %.6f m, ", path_length);
end


function master_table = update_master_table( ...
    master_table, subject_id, shape_id, controller_id, beta, K, R2, RMSE)
    %UPDATE_MASTER_TABLE Upsert one power-law result row.

    if height(master_table) == 0
        master_table = table(subject_id, shape_id, controller_id, ...
            beta, K, R2, RMSE, 'VariableNames', ...
            {'subject_id', 'shape_id', 'controller_id', ...
             'beta', 'K', 'R2', 'RMSE'});
        return;
    end

    if ~ismember("beta", string(master_table.Properties.VariableNames))
        master_table.beta = nan(height(master_table), 1);
    end

    if ~ismember("K", string(master_table.Properties.VariableNames))
        master_table.K = nan(height(master_table), 1);
    end

    if ~ismember("R2", string(master_table.Properties.VariableNames))
        master_table.R2 = nan(height(master_table), 1);
    end

    if ~ismember("RMSE", string(master_table.Properties.VariableNames))
        master_table.RMSE = nan(height(master_table), 1);
    end

    row_idx = master_table.subject_id == subject_id & ...
              master_table.shape_id == shape_id & ...
              master_table.controller_id == controller_id;

    if any(row_idx)
        master_table.beta(row_idx) = beta;
        master_table.K(row_idx) = K;
        master_table.R2(row_idx) = R2;
        master_table.RMSE(row_idx) = RMSE;
    else
        new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                    controller_id, beta, K, R2, RMSE);
        master_table = [master_table; new_row];
    end
end


function new_row = make_appended_row( ...
    master_table, subject_id, shape_id, controller_id, beta, K, R2, RMSE)
    %MAKE_APPENDED_ROW Create one row with the master table schema.

    new_row = master_table(1, :);

    for var_idx = 1:width(new_row)
        var_name = new_row.Properties.VariableNames{var_idx};
        new_row.(var_name) = get_missing_value(new_row.(var_name));
    end

    new_row.subject_id = subject_id;
    new_row.shape_id = shape_id;
    new_row.controller_id = controller_id;
    new_row.beta = beta;
    new_row.K = K;
    new_row.R2 = R2;
    new_row.RMSE = RMSE;
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
