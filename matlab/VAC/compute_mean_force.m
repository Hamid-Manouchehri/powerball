% Compute mean force for one VAC dataset and update the matching row.
%
% This script reads one subject/shape/controller trial, computes one mean
% planar force magnitude, and writes only the matching row in the master
% CSV. Existing rows and columns are preserved.
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
% read_csv_file = "./raw_data/eight_chen_var_adm_schunk.csv";  % TODO
% read_csv_file = "./raw_data/eight_kang_indirect_var_adm_schunk.csv";  % TODO
subject_id = 1;       % TODO [sub1:1]
shape_id = 1;         % TODO [eight_sign:1, ellipse:2, four_leaves:3, squircle:4]
controller_id = 2;    % TODO [chen:1, kang_indirect:2]

write_master_csv_file = "./processed_data/master_dataset.csv";

% -------------------- Read Raw Trial --------------------
input_table = readtable(read_csv_file);

force_xy = sqrt(input_table.FT1.^2 + input_table.FT2.^2);
mean_force = mean(force_xy);

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

    if ~ismember("mean_force", string(master_table.Properties.VariableNames))
        master_table.mean_force = nan(height(master_table), 1);
    end

    if any(row_mask)
        master_table.mean_force(row_mask) = mean_force;
    else
        if height(master_table) == 0
            master_table = table(subject_id, shape_id, controller_id, ...
                mean_force, 'VariableNames', ...
                {'subject_id', 'shape_id', 'controller_id', 'mean_force'});
        else
            new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                        controller_id, mean_force);
            master_table = [master_table; new_row];
        end
    end
else
    master_table = table(subject_id, shape_id, controller_id, ...
        mean_force, 'VariableNames', ...
        {'subject_id', 'shape_id', 'controller_id', 'mean_force'});
end

writetable(master_table, write_master_csv_file);

fprintf("\nMean force updated in:\n");
fprintf("%s\n", write_master_csv_file);
fprintf("subject_id = %d, shape_id = %d, controller_id = %d\n", ...
    subject_id, shape_id, controller_id);
fprintf("mean_force = %.10f\n", mean_force);
fprintf("Rows preserved = %d\n", height(master_table));


function new_row = make_appended_row(master_table, subject_id, shape_id, ...
                                     controller_id, mean_force)
    %MAKE_APPENDED_ROW Create one row with the master table schema.

    new_row = master_table(1, :);

    for var_idx = 1:width(new_row)
        var_name = new_row.Properties.VariableNames{var_idx};
        new_row.(var_name) = get_missing_value(new_row.(var_name));
    end

    new_row.subject_id = subject_id;
    new_row.shape_id = shape_id;
    new_row.controller_id = controller_id;
    new_row.mean_force = mean_force;
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
