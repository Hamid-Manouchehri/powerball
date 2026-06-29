% Compute duration for one VAC dataset and update the matching master row.
%
% This script reads one subject/shape/controller trial, computes the
% movement duration, and writes only the matching row in the master CSV.
% Existing rows and columns are preserved.
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
% read_csv_file = "./raw_data/test_four_leaves_chen_var_adm_schunk.csv";  % TODO
% read_csv_file = "./raw_data/test_four_leaves_chen_kf_var_adm_schunk.csv";  % TODO

subject_id = 1;       % TODO [sub1:1]
shape_id = 3;         % TODO [eight_sign:1, ellipse:2, four_leaves:3, squircle:4]
controller_id = 4;    % TODO [chen:1, kang_indirect:2, chen_ls:3, chen_kf:4]

% write_master_csv_file = "./processed_data/master_dataset.csv";
write_master_csv_file = "./processed_data/master_dataset_chen_ls_kf_study.csv";

% -------------------- Read Raw Trial --------------------
input_table = readtable(read_csv_file);

t = (input_table.Time_us - input_table.Time_us(1)) / 1e6;
duration = t(end) - t(1);

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

    if ~ismember("duration", string(master_table.Properties.VariableNames))
        master_table.duration = nan(height(master_table), 1);
    end

    if any(row_mask)
        master_table.duration(row_mask) = duration;
    else
        if height(master_table) == 0
            master_table = table(subject_id, shape_id, controller_id, ...
                duration, 'VariableNames', ...
                {'subject_id', 'shape_id', 'controller_id', 'duration'});
        else
            new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                        controller_id, duration);
            master_table = [master_table; new_row];
        end
    end
else
    master_table = table(subject_id, shape_id, controller_id, duration, ...
        'VariableNames', {'subject_id', 'shape_id', 'controller_id', ...
        'duration'});
end

writetable(master_table, write_master_csv_file);

fprintf("\nDuration updated in:\n");
fprintf("%s\n", write_master_csv_file);
fprintf("subject_id = %d, shape_id = %d, controller_id = %d\n", ...
    subject_id, shape_id, controller_id);
fprintf("duration = %.10f\n", duration);
fprintf("Rows preserved = %d\n", height(master_table));


function new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                     controller_id, duration)
    %MAKE_APPENDED_ROW Create one row with the master table schema.

    new_row = master_table(1, :);

    for var_idx = 1:width(new_row)
        var_name = new_row.Properties.VariableNames{var_idx};
        new_row.(var_name) = get_missing_value(new_row.(var_name));
    end

    new_row.subject_id = subject_id;
    new_row.shape_id = shape_id;
    new_row.controller_id = controller_id;
    new_row.duration = duration;
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
