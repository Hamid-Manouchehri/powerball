% Export selected columns from one or more CSV files into one master CSV.
%
% This script reads input CSV files, keeps only the selected columns,
% optionally renames those columns, and writes the combined result to a
% master dataset CSV file.
%
% Inputs:
%   read_csv_files: one or more CSV file paths
%   read_selected_columns: column names to extract from each CSV
%
% Outputs:
%   write_master_csv_file: combined CSV file with the selected columns

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_csv_files = [
    "chen_paper_implementation/raw_data/" + ...
    "eight_sign_chen_var_adm_schunk.csv"      % TODO input CSV
    "chen_paper_implementation/raw_data/" + ...
    "ellipse_chen_var_adm_schunk.csv"         % TODO input CSV
];

read_selected_columns = [
    "Time_us"     % TODO column name in input CSV
    "X"           % TODO column name in input CSV
    "Y"           % TODO column name in input CSV
    "FT1"         % TODO column name in input CSV
    "FT2"         % TODO column name in input CSV
];

write_output_columns = [
    "Time_us"     % TODO output column name
    "X"           % TODO output column name
    "Y"           % TODO output column name
    "Fx"          % TODO output column name
    "Fy"          % TODO output column name
];

write_master_csv_file = "master_dataset.csv";   % TODO output CSV file
append_source_file_name = true;                 % TODO true or false
overwrite_master_file = true;                   % TODO true or false

% -------------------- Read And Merge --------------------
master_table = table();

for file_idx = 1:numel(read_csv_files)
    read_csv_file = read_csv_files(file_idx);
    input_table = readtable(read_csv_file);

    selected_table = input_table(:, cellstr(read_selected_columns));
    selected_table.Properties.VariableNames = ...
        cellstr(write_output_columns);

    if append_source_file_name
        selected_table.SourceFile = repmat(read_csv_file, ...
            height(selected_table), 1);
    end

    master_table = [master_table; selected_table];
end

% -------------------- Write Master CSV --------------------
if overwrite_master_file
    writetable(master_table, write_master_csv_file);
else
    if isfile(write_master_csv_file)
        old_master_table = readtable(write_master_csv_file);
        master_table = [old_master_table; master_table];
    end

    writetable(master_table, write_master_csv_file);
end

fprintf("\nMaster dataset written to:\n");
fprintf("%s\n", write_master_csv_file);
fprintf("Total rows = %d\n", height(master_table));
fprintf("Total columns = %d\n", width(master_table));
