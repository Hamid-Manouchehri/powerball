% Plot VAC metrics grouped by controller.
%
% This script reads the master metric CSV and creates one figure with all
% requested metrics. Each subplot compares the metric values by controller.
%
% Inputs:
%   read_master_csv_file: master dataset with metric columns
%
% Outputs:
%   Figure:
%   1) DSJ by controller
%   2) mean force by controller
%   3) path error by controller
%   4) boundary violation by controller
%   5) beta by controller
%   6) K by controller
%   7) R2 of power-law fit by controller

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_master_csv_file = "./processed_data/master_dataset.csv";  % TODO
read_ch_csv_file = "./raw_data/ellipse_chen_var_adm_schunk.csv";  % TODO
read_kg_csv_file = "./raw_data/ellipse_kang_indirect_var_adm_schunk.csv";  % TODO

ch_dataset = readtable(read_ch_csv_file);
kg_dataset = readtable(read_kg_csv_file);

controller_ids = [1, 2];                % TODO controller numeric IDs
controller_names = ["Chen", "Kang"];    % TODO controller plot labels
shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO

metric_columns = [
    "DSJ"                       % TODO DSJ column name
    "mean_force"                % TODO mean force column name
    "path_length_error"         % TODO path error column name
    "boundary_violation_count"  % TODO boundary violation column name
    % "beta"                      % TODO beta column name
    % "K"                         % TODO K column name
    % "R2"                        % TODO power-law R2 column name
];

metric_titles = [
    "DSJ by Controller"
    "Mean Force by Controller"
    "Path Error by Controller"
    "Boundary Violation by Controller"
    % "Beta by Controller"
    % "K by Controller"
    % "Power-law R2 by Controller"
];

metric_ylabels = [
    "DSJ [-]"
    "Mean force [N]"
    "Path error [m]"
    "Boundary violation count [-]"
    "Beta [-]"
    "K"
    "R2 [-]"
];

% -------------------- Read Master Dataset --------------------
master_dataset = readtable(read_master_csv_file);
controller_group = categorical(master_dataset.controller_id, ...
                               controller_ids, controller_names);

% -------------------- Plot Metrics --------------------
figure(Name="vac_metrics_by_controller", NumberTitle="off");
tiledlayout(3, 3, "TileSpacing", "compact", "Padding", "compact");

for metric_idx = 1:numel(metric_columns)
    metric_column = metric_columns(metric_idx);
    metric_value = master_dataset.(metric_column);

    nexttile;
    boxchart(controller_group, metric_value);
    hold on;
    swarmchart(controller_group, metric_value, 28, 'filled');
    grid on;

    xlabel("Controller");
    ylabel(metric_ylabels(metric_idx));
    title(metric_titles(metric_idx));
end

