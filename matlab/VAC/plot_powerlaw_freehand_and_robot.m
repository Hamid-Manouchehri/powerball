% Plot whole-shape power-law beta for robot and free-hand data.
%
% This script reads the robot master dataset and the free-hand power-law
% dataset, then compares beta values for each shape in one grouped bar
% chart.
%
% Inputs:
%   ./processed_data/master_dataset.csv
%   ./processed_data/free_hand_powerlaw.csv
%
% Outputs:
%   Figure:
%   1) grouped bar chart of beta for Chen, Kang, and free hand

clc;
clear;
% close all;

robot_table = readtable("./processed_data/master_dataset.csv");
free_table = readtable("./processed_data/free_hand_powerlaw.csv");

shape_names = ["eight", "ellipse", "clover", "squircle"];
controller_ids = [1, 2];
controller_names = ["Chen", "Kang", "Free hand"];

beta_values = nan(numel(shape_names), numel(controller_names));

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_ids)
        row_idx = robot_table.shape_id == shape_idx & ...
            robot_table.controller_id == controller_ids(controller_idx);

        beta_values(shape_idx, controller_idx) = robot_table.beta(row_idx);
    end

    beta_values(shape_idx, 3) = free_table.beta(shape_idx);
end

figure(Name="beta_robot_vs_free_hand", NumberTitle="off");
bar(categorical(shape_names), beta_values, "grouped");
grid on;

xlabel("Shape");
ylabel("\beta [-]");
title("Power-Law Beta: Robot Controllers vs Free Hand");
legend(controller_names, "Location", "best");
yline(1/3, "r-.");
