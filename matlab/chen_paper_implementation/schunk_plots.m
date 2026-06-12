% Plot Schunk trajectory, velocity heatmap, force, and damping.
%
% Inputs:
%   read_csv_file: CSV file written by the controller log.
%   read_force_column: force column name in the CSV.
%   read_damping_column: damping column name in the CSV.
%
% Outputs:
%   Figure 1:
%   1) Cartesian end-effector trajectory [m]
%   2) trajectory colored by planar speed [m/s]
%   Figure 2:
%   1) force magnitude [N]
%   2) damping [Ns/m]
%   Figure 3:
%   1) beta exponent [-]
%   2) K parameter
%   Command window:
%   1) dimensionless squared jerk (DSJ)
%   2) mean planar force [N]

clc;
clear;
close all;

addpath("functions/");

% -------------------- User Settings --------------------
read_csv_file = "data/test_exp_chen_var_adm_schunk.csv";  % TODO
schunk_table = readtable(read_csv_file);

t = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;  % second
Q      = schunk_table{:,2:7};
Qdot_a = schunk_table{:,8:13};   % Schunk angular velocity
Qdot   = schunk_table{:,14:19};  % Input velocity command
FT     = schunk_table{:,20:25};  % Raw FT
F_cmd  = schunk_table{:,26:31};  % FT after filtering
v_meas = schunk_table{:,32:37};  % end-effector velocity
vel    = schunk_table{:,38:43};  % admittance vel
Ci     = schunk_table{:,53:58};  % virtual damping
beta_hat = schunk_table.beta_hat;
K_hat    = schunk_table.K_hat;

ee_pos    = zeros(length(t),3);
ee_orient = zeros(length(t),3);
ee_vel    = zeros(length(t),6);

% Extracting cartesian space data from schunk:
for i = 1:length(t)
    ee_vel(i, :) = Qdot_a(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    % [yaw pitch roll] in radians
    ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');
    ee_pos(i, :) = T(4, :);
end

speed_xy = sqrt(ee_vel(:,1).^2 + ee_vel(:,2).^2);  % m/s

% Chen paper metric:
% DSJ = integral(jerk_x^2 + jerk_y^2) dt * T^5 / A^2
% T is movement time [s], and A is planar path length [m].
x = ee_pos(:,1);
y = ee_pos(:,2);
movement_time = t(end) - t(1);
step_length = sqrt(diff(x).^2 + diff(y).^2);
path_length = sum(step_length);

vx_dsj = gradient(x, t);
vy_dsj = gradient(y, t);
ax_dsj = gradient(vx_dsj, t);
ay_dsj = gradient(vy_dsj, t);
jx_dsj = gradient(ax_dsj, t);
jy_dsj = gradient(ay_dsj, t);

jerk_squared = jx_dsj.^2 + jy_dsj.^2;
dsj = trapz(t, jerk_squared) * movement_time^5 / path_length^2;

force_xy = sqrt(FT(:,1).^2 + FT(:,2).^2);  % N
mean_force = mean(force_xy);

fprintf("DSJ = %.6g\n", dsj);
fprintf("Mean planar force = %.6f N\n", mean_force);

figure(Name="cartesian_trajectory", NumberTitle="off");

% subplot(1,2,1);
% builtin('plot', ee_pos(:,1), ee_pos(:,2), 'b-', 'LineWidth', 1.2);
% axis equal;
% grid on;
% xlabel("x [m]");
% ylabel("y [m]");
% title("End-effector trajectory");

% subplot(1,2,2);
scatter(ee_pos(:,1), ee_pos(:,2), 18, speed_xy, 'filled');
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Velocity heatmap over trajectory");
colormap jet;
cb = colorbar;
cb.Label.String = "speed [m/s]";

% figure(Name="force_damping", NumberTitle="off");
% 
% subplot(2,1,1);
% builtin('plot', t, sqrt(FT(:,1).^2 + FT(:,2).^2), 'LineWidth', 1.2);
% grid on;
% ylabel("Force [N]");
% title("Force and damping");
% 
% subplot(2,1,2);
% builtin('plot', t, sqrt(Ci(:,1).^2 + Ci(:,2).^2), 'LineWidth', 1.2);
% grid on;
% xlabel("Time [s]");
% ylabel("Damping [Ns/m]");

figure(Name="beta_K", NumberTitle="off");

subplot(2,1,1);
builtin('plot', t, beta_hat, 'LineWidth', 1.2);
grid on;
ylabel("\beta [-]");
title("Power-law parameters");

subplot(2,1,2);
builtin('plot', t, K_hat, 'LineWidth', 1.2);
grid on;
xlabel("Time [s]");
ylabel("K");
