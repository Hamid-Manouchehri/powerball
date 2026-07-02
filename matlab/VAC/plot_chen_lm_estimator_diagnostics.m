% Plot Chen-LM variable admittance controller estimator diagnostics.
%
% This script reads one CSV log recorded from src_main/chen_lm_var_adm.cpp.
% It focuses on checking whether the Levenberg-Marquardt estimator is
% behaving well over time.
%
% Inputs:
%   read_csv_file: robot raw CSV with LM diagnostic columns.
%
% Outputs:
%   Figures:
%   1) XY trajectory colored by speed.
%   2) beta_hat, K_hat, curvature_hat, and target_curvature over time.
%   3) LM cost, cost reduction, lambda, and step norm over time.
%   4) LM valid samples, iterations, R2, RMSE, and absolute error.
%   5) speed, force magnitude, damping, and intended motion signals.

clc;
clear;
close all;

% -------------------- User Settings --------------------
read_csv_file = "./raw_data/test_lm_chen_lm_var_adm_schunk.csv";  % TODO

trajectory_speed_limits = [0, 0.20];  % TODO speed color range [m/s]
beta_ylim = [0, 0.8];                 % TODO beta_hat y range [-]
K_ylim = [0, 0.5];                    % TODO K_hat/VGF y range
curvature_ylim = [0, 200];            % TODO curvature y range [1/m]
R2_ylim = [0, 1];                     % TODO R2 y range [-]

beta_histogram_edges = 0.0:0.01:0.8;  % TODO beta_hat histogram bins
K_histogram_edges = 0.0:0.01:0.5;     % TODO K_hat histogram bins

% -------------------- Read Data --------------------
raw_table = readtable(read_csv_file);
t = (raw_table.Time_us - raw_table.Time_us(1)) * 1e-6;

speed_xy = sqrt(raw_table.v_meas1.^2 + raw_table.v_meas2.^2);
force_xy = sqrt(raw_table.FT1.^2 + raw_table.FT2.^2);

% -------------------- Print Quick Summary --------------------
fprintf("\nChen-LM estimator diagnostic summary\n");
fprintf("File: %s\n", read_csv_file);
fprintf("Duration = %.3f s\n", t(end) - t(1));
fprintf("Mean beta_hat = %.6f\n", mean(raw_table.beta_hat));
fprintf("Mean K_hat/VGF = %.6f\n", mean(raw_table.K_hat));
fprintf("Mean R2 = %.6f\n", mean(raw_table.power_law_R2));
fprintf("Mean RMSE = %.6f\n", mean(raw_table.power_law_RMSE));
fprintf("Mean LM final cost = %.8f\n", mean(raw_table.lm_cost_final));
fprintf("Mean LM abs error = %.8f m/s\n", ...
    mean(raw_table.lm_mean_abs_error));

% -------------------- Plot XY Trajectory With Speed Heatmap ----------------
figure(Name="chen_lm_xy_speed_heatmap", NumberTitle="off");
scatter(raw_table.X, raw_table.Y, 8, speed_xy, "filled");
grid on;
axis equal;
colormap(turbo);
clim(trajectory_speed_limits);
colorbar;
xlabel("X [m]");
ylabel("Y [m]");
title("Chen-LM XY Trajectory Colored by Speed");

% -------------------- Plot Power-Law Estimates --------------------
figure(Name="chen_lm_power_law_estimates", NumberTitle="off");
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, raw_table.beta_hat, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("\beta_{hat} [-]");
title("Estimated Beta");
ylim(beta_ylim);

nexttile;
plot(t, raw_table.K_hat, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("K_{hat} / VGF");
title("Estimated Velocity Gain Factor");
ylim(K_ylim);

nexttile;
plot(t, raw_table.curvature_hat, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("\kappa_{hat} [1/m]");
title("Estimated Curvature");
ylim(curvature_ylim);

nexttile;
plot(t, raw_table.target_curvature, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("\kappa_{target} [1/m]");
title("Target Curvature");
ylim(curvature_ylim);

% -------------------- Plot LM Cost And Convergence Signals --------------------
figure(Name="chen_lm_cost_and_convergence", NumberTitle="off");
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, raw_table.lm_cost_initial, "LineWidth", 1.0);
hold on;
plot(t, raw_table.lm_cost_final, "LineWidth", 1.0);
grid on;
xlabel("Time [s]");
ylabel("Cost [(m/s)^2]");
title("LM Cost Before and After Update");
legend(["Initial", "Final"], "Location", "best");

nexttile;
plot(t, raw_table.lm_cost_reduction, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Cost reduction [(m/s)^2]");
title("LM Cost Reduction");

nexttile;
plot(t, raw_table.lm_lambda_final, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("\lambda [-]");
title("Final LM Damping Lambda");

nexttile;
plot(t, raw_table.lm_step_norm_final, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Step norm [-]");
title("Final LM Parameter Step Norm");

% -------------------- Plot Fit Quality And Errors --------------------
figure(Name="chen_lm_fit_quality_and_errors", NumberTitle="off");
tiledlayout(3, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, raw_table.lm_valid_count, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Samples [-]");
title("Valid Samples in LM Window");

nexttile;
plot(t, raw_table.lm_iterations_used, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Iterations [-]");
title("LM Iterations Used");

nexttile;
plot(t, raw_table.power_law_R2, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("R2 [-]");
title("Power-Law R2");
ylim(R2_ylim);

nexttile;
plot(t, raw_table.power_law_RMSE, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("RMSE [m/s]");
title("Power-Law RMSE");

nexttile;
plot(t, raw_table.lm_mean_abs_error, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Mean abs error [m/s]");
title("LM Mean Absolute Speed Error");

nexttile;
plot(t, raw_table.lm_max_abs_error, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Max abs error [m/s]");
title("LM Max Absolute Speed Error");

% -------------------- Plot Controller And Interaction Signals --------------
figure(Name="chen_lm_controller_signals", NumberTitle="off");
tiledlayout(3, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(t, speed_xy, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Speed XY [m/s]");
title("Robot Speed");

nexttile;
plot(t, force_xy, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("|F_{xy}| [N]");
title("External Force Magnitude");

nexttile;
plot(t, raw_table.bd_desired, "LineWidth", 1.0);
hold on;
plot(t, raw_table.bd_applied, "LineWidth", 1.0);
grid on;
xlabel("Time [s]");
ylabel("Damping [Ns/m]");
title("Desired and Applied Damping");
legend(["bd desired", "bd applied"], "Location", "best");

nexttile;
plot(t, raw_table.intended_acc_x, "LineWidth", 1.0);
hold on;
plot(t, raw_table.intended_acc_y, "LineWidth", 1.0);
grid on;
xlabel("Time [s]");
ylabel("Acceleration [m/s^2]");
title("Intended Acceleration");
legend(["x", "y"], "Location", "best");

nexttile;
plot(t, raw_table.intended_force_x, "LineWidth", 1.0);
hold on;
plot(t, raw_table.intended_force_y, "LineWidth", 1.0);
grid on;
xlabel("Time [s]");
ylabel("Force [N]");
title("Intended Force");
legend(["x", "y"], "Location", "best");

nexttile;
plot(t, raw_table.indirect_ready, "LineWidth", 1.1);
grid on;
xlabel("Time [s]");
ylabel("Ready [-]");
title("Indirect Estimator Ready Flag");
ylim([-0.1, 1.1]);

% -------------------- Plot Estimate Histograms --------------------
figure(Name="chen_lm_beta_K_histograms", NumberTitle="off");
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
histogram(raw_table.beta_hat, beta_histogram_edges);
grid on;
xlabel("\beta_{hat} [-]");
ylabel("Sample count");
title("Beta Estimate Histogram");
xlim([beta_histogram_edges(1), beta_histogram_edges(end)]);

nexttile;
histogram(raw_table.K_hat, K_histogram_edges);
grid on;
xlabel("K_{hat} / VGF");
ylabel("Sample count");
title("VGF Estimate Histogram");
xlim([K_histogram_edges(1), K_histogram_edges(end)]);
