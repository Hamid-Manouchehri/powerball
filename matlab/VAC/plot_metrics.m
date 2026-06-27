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
%   8) RMSE of power-law fit by controller
%   9) DSJ heatmap over XY trajectory

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_master_csv_file = "./processed_data/master_dataset.csv";  % TODO
read_free_hand_csv_file = "./processed_data/free_hand_powerlaw.csv";  % TODO
read_raw_data_dir = "./raw_data/";  % TODO raw controller CSV folder

controller_ids = [1, 2];                % TODO controller numeric IDs
controller_names = ["Chen", "Kang"];    % TODO controller plot labels

controller_file_names = [
    "chen_var_adm_schunk"           % TODO Chen filename suffix
    "kang_indirect_var_adm_schunk"  % TODO Kang filename suffix
    ];

shape_names = ["eight", "ellipse", "clover", "squircle"];  % TODO shapes

beta_xlim = [0, 50];          % TODO beta plot time range [s]
beta_ylim = [0, 0.7];    % TODO beta range [-]
K_xlim = [0, 50];             % TODO K plot time range [s]
K_ylim = [0, 0.30];            % TODO K range
trajectory_speed_limits = [0, 0.20];  % TODO speed color range [m/s]
R2_ylim = [0, 1];                      % TODO R2 axis range [-]
RMSE_ylim = [0, 0.5];                  % TODO RMSE axis range
DSJ_scale = 1e9;                       % TODO divide DSJ in heatmap

metric_columns = [
    "DSJ"                       % TODO DSJ column name
    "mean_force"                % TODO mean force column name
    "path_length_error"         % TODO path error column name
    "boundary_violation_count"  % TODO boundary violation column name
    ];

metric_titles = [
    "DSJ by Controller"
    "Mean Force by Controller"
    "Path Error by Controller"
    "Boundary Violation by Controller"
    ];

metric_ylabels = [
    "DSJ [-]"
    "Mean force [N]"
    "Path error [m]"
    "Boundary violation count [-]"
    ];

% -------------------- Read Master Dataset --------------------
master_dataset = readtable(read_master_csv_file);
free_hand_dataset = readtable(read_free_hand_csv_file);
controller_group = categorical(master_dataset.controller_id, ...
    controller_ids, controller_names);
shape_group = categorical(master_dataset.shape_id, ...
    1:numel(shape_names), shape_names);


% -------------------- Plot Trajectories With Speed Heatmap --------------------
figure(Name="trajectory_speed_heatmap_by_shape_controller", ...
    NumberTitle="off");
tiledlayout(numel(controller_names), numel(shape_names), ...
    "TileSpacing", "compact", "Padding", "compact");

for controller_idx = 1:numel(controller_names)
    for shape_idx = 1:numel(shape_names)
        read_raw_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
            "_" + controller_file_names(controller_idx) + ".csv";

        raw_dataset = readtable(read_raw_csv_file);
        speed_xy = sqrt(raw_dataset.v_meas1.^2 + raw_dataset.v_meas2.^2);

        nexttile;
        scatter(raw_dataset.X, raw_dataset.Y, 8, speed_xy, 'filled');
        grid on;
        axis equal;
        clim(trajectory_speed_limits);

        title(shape_names(shape_idx) + " " + controller_names(controller_idx));
        xlabel("X [m]");
        ylabel("Y [m]");
    end
end

colormap turbo;
colorbar;

% -------------------- Plot Duration --------------------
duration_values = nan(numel(shape_names), numel(controller_names));

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_names)
        row_idx = master_dataset.shape_id == shape_idx & ...
            master_dataset.controller_id == ...
            controller_ids(controller_idx);

        duration_values(shape_idx, controller_idx) = ...
            mean(master_dataset.duration(row_idx));
    end
end

figure(Name="duration_by_shape_and_controller", NumberTitle="off");
bar(categorical(shape_names), duration_values, "grouped");
grid on;

xlabel("Shape");
ylabel("Duration [s]");
title("Duration by Shape and Controller");
legend(controller_names, "Location", "best");

% -------------------- Plot Power-Law Fit Quality --------------------
R2_values = nan(numel(shape_names), numel(controller_names));
RMSE_values = nan(numel(shape_names), numel(controller_names));

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_names)
        row_idx = master_dataset.shape_id == shape_idx & ...
            master_dataset.controller_id == ...
            controller_ids(controller_idx);

        R2_values(shape_idx, controller_idx) = ...
            mean(master_dataset.R2(row_idx));
        RMSE_values(shape_idx, controller_idx) = ...
            mean(master_dataset.RMSE(row_idx));
    end
end

figure(Name="power_law_fit_quality_by_shape_controller", ...
    NumberTitle="off");
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
bar(categorical(shape_names), R2_values, "grouped");
grid on;
xlabel("Shape");
ylabel("R2 [-]");
title("Power-Law R2");
ylim(R2_ylim);
legend(controller_names, "Location", "best");

nexttile;
bar(categorical(shape_names), RMSE_values, "grouped");
grid on;
xlabel("Shape");
ylabel("RMSE [-]");
title("Power-Law RMSE");
ylim(RMSE_ylim);
legend(controller_names, "Location", "best");


% -------------------- Plot Metrics --------------------
figure(Name="vac_metrics_by_controller", NumberTitle="off");
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

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

% -------------------- Plot Whole-Shape Beta --------------------

controller_ids = [1, 2];
controller_and_free_hand_names = ["Chen", "Kang", "Free hand"];

beta_values = nan(numel(shape_names), numel(controller_and_free_hand_names));

for shape_idx = 1:numel(shape_names)
    for controller_idx = 1:numel(controller_ids)
        row_idx = master_dataset.shape_id == shape_idx & ...
            master_dataset.controller_id == controller_ids(controller_idx);

        beta_values(shape_idx, controller_idx) = master_dataset.beta(row_idx);
    end

    beta_values(shape_idx, 3) = free_hand_dataset.beta(shape_idx);
end

figure(Name="beta_robot_vs_free_hand", NumberTitle="off");
bar(categorical(shape_names), beta_values, "grouped");
grid on;

xlabel("Shape");
ylabel("\beta [-]");
title("Whole-Shpae Power-Law Beta: Robot Controllers vs Free Hand");
legend(controller_and_free_hand_names, "Location", "best");
yline(1/3, "r-.");


% -------------------- Plot Beta Profiles --------------------
figure(Name="beta_hat_profiles_by_shape_controller", NumberTitle="off");
tiledlayout(numel(controller_names), numel(shape_names), ...
    "TileSpacing", "compact", "Padding", "compact");

for controller_idx = 1:numel(controller_names)
    for shape_idx = 1:numel(shape_names)
        read_raw_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
            "_" + controller_file_names(controller_idx) + ".csv";

        raw_dataset = readtable(read_raw_csv_file);
        t = (raw_dataset.Time_us - raw_dataset.Time_us(1)) / 1e6;

        nexttile;
        %plot(t, raw_dataset.beta_hat, "LineWidth", 1.1);
        plot(raw_dataset.beta_hat(1:end-1),raw_dataset.beta_hat(2:end),'.');
        hold on
        plot(raw_dataset.beta_hat,raw_dataset.beta_hat,'r');
        b = raw_dataset.beta_hat;
        rho1 = corr(b(1:end-1), b(2:end), 'rows', 'complete')
        %histogram(raw_dataset.beta_hat, 0.02:0.001:0.7)
        %        grid on;
        %numZ = sum(raw_dataset.beta_hat==0)/length(raw_dataset.beta_hat);
        %title(num2str(numZ)) 
        %       title(shape_names(shape_idx) + " " + controller_names(controller_idx));
        %      xlabel("Time [s]");
        %     ylabel("\beta_{hat} [-]");
        %    xlim(beta_xlim);
        %   ylim(beta_ylim);

    end
end

% -------------------- Plot K Profiles --------------------
% figure(Name="K_hat_profiles_by_shape_controller", NumberTitle="off");
% tiledlayout(numel(controller_names), numel(shape_names), ...
%     "TileSpacing", "compact", "Padding", "compact");
% 
% for controller_idx = 1:numel(controller_names)
%     for shape_idx = 1:numel(shape_names)
%         read_raw_csv_file = read_raw_data_dir + shape_names(shape_idx) + ...
%             "_" + controller_file_names(controller_idx) + ".csv";
% 
%         raw_dataset = readtable(read_raw_csv_file);
%         t = (raw_dataset.Time_us - raw_dataset.Time_us(1)) / 1e6;
% 
%         nexttile;
%         plot(t, raw_dataset.K_hat, "LineWidth", 1.1);
%         grid on;
% 
%         title(shape_names(shape_idx) + " " + controller_names(controller_idx));
%         xlabel("Time [s]");
%         ylabel("K_{hat}");
%         xlim(K_xlim);
%         ylim(K_ylim);
%     end
% end
