% Compute DSJ for one VAC dataset and update the matching master row.
%
% This script reads one subject/shape/controller trial, computes the
% dimensionless squared jerk (DSJ), and writes only the matching row in
% the master CSV. Existing rows and columns are preserved.
%
% Inputs:
%   read_csv_file: raw trial CSV file
%   subject_id: subject identifier used in master CSV
%   shape_id: shape identifier used in master CSV
%   controller_id: controller identifier used in master CSV
%
% Outputs:
%   write_master_csv_file: updated master dataset CSV

clc;
clear;
close all;

% -------------------- User Settings --------------------
% read_csv_file = "./raw_data/squircle_chen_var_adm_schunk.csv";  % TODO
% read_csv_file = "./raw_data/squircle_kang_indirect_var_adm_schunk.csv";  % TODO
subject_id = 1;       % TODO [sub1:1]
shape_id = 4;         % TODO [eight_sign:1, ellipse:2, four_leaves:3, squircle:4]
controller_id = 2;    % TODO [chen:1, kang_indirect:2]

write_master_csv_file = "./processed_data/master_dataset.csv";

% -------------------- Read Raw Trial --------------------
input_table = readtable(read_csv_file);

t = (input_table.Time_us - input_table.Time_us(1)) / 1e6;
movement_time = t(end) - t(1);

step_length = sqrt(diff(input_table.X).^2 + diff(input_table.Y).^2);
path_length = sum(step_length);

vx_dsj = input_table.v_meas1;
vy_dsj = input_table.v_meas2;
ax_dsj = gradient(vx_dsj, t);
ay_dsj = gradient(vy_dsj, t);
jx_dsj = gradient(ax_dsj, t);
jy_dsj = gradient(ay_dsj, t);

jerk_squared = jx_dsj.^2 + jy_dsj.^2;
dsj = trapz(t, jerk_squared) * movement_time^5 / path_length^2;

% -------------------- Write/Update Master CSV --------------------
if isfile(write_master_csv_file)
    master_table = readtable(write_master_csv_file);

    required_variables = ["subject_id", "shape_id", "controller_id"];
    if ~all(ismember(required_variables, ...
                     string(master_table.Properties.VariableNames)))
        error("master CSV must contain subject_id, shape_id, " + ...
              "and controller_id");
    end

    row_mask = master_table.subject_id == subject_id & ...
               master_table.shape_id == shape_id & ...
               master_table.controller_id == controller_id;

    if ~ismember("DSJ", string(master_table.Properties.VariableNames))
        master_table.DSJ = nan(height(master_table), 1);
    end

    if any(row_mask)
        master_table.DSJ(row_mask) = dsj;
    else
        if height(master_table) == 0
            master_table = table(subject_id, shape_id, controller_id, dsj, ...
                'VariableNames', ...
                {'subject_id', 'shape_id', 'controller_id', 'DSJ'});
        else
            new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                        controller_id, dsj);
            master_table = [master_table; new_row];
        end
    end
else
    master_table = table(subject_id, shape_id, controller_id, dsj, ...
        'VariableNames', {'subject_id', 'shape_id', 'controller_id', 'DSJ'});
end

writetable(master_table, write_master_csv_file);

fprintf("\nDSJ updated in:\n");
fprintf("%s\n", write_master_csv_file);
fprintf("subject_id = %d, shape_id = %d, controller_id = %d\n", ...
    subject_id, shape_id, controller_id);
fprintf("DSJ = %.10f\n", dsj);
fprintf("Rows preserved = %d\n", height(master_table));


function new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                     controller_id, dsj)
    %MAKE_APPENDED_ROW Create one row with the master table schema.

    new_row = master_table(1, :);

    for var_idx = 1:width(new_row)
        var_name = new_row.Properties.VariableNames{var_idx};
        new_row.(var_name) = get_missing_value(new_row.(var_name));
    end

    new_row.subject_id = subject_id;
    new_row.shape_id = shape_id;
    new_row.controller_id = controller_id;
    new_row.DSJ = dsj;
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
