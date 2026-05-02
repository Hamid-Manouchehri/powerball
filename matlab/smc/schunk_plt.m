clear; clc; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/smc/functions/")

read_schunk_file = "S2_var_clover_schunk.csv";  % TODO
save_schunk_csv = "test.csv";  % TODO

csvDir = "/home/hamid-tuf/projects/powerball/matlab/smc/data/schunk_data/";


read_schunk_file = csvDir + read_schunk_file;
schunk_table = readtable(read_schunk_file);

t          = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
Q          = schunk_table{:,2:7};
Qdot_a     = schunk_table{:,8:13};   % Schunk angular velocity
Qdot       = schunk_table{:,14:19};  % Input velocity command (admittance control)
FT         = schunk_table{:,20:25};  % Raw FT
F_cmd      = schunk_table{:,26:31};  % FT after filtering
v_meas     = schunk_table{:,32:37};  % end-effector velocity
v_meas_lpf = schunk_table{:,38:43};  % end-effector velocity
vel        = schunk_table{:,44:49};  % admittance vel
c_des      = schunk_table{:,50:51};  % end-effector velocity
Cd         = schunk_table{:,52:57};  % admittance vel

schunk_numOfDataSamples = size(t, 1);

ee_pos    = zeros(schunk_numOfDataSamples,3);
ee_orient = zeros(schunk_numOfDataSamples,3);
ee_vel    = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qdot_a(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

figure(Name="ee_pos",NumberTitle="off");
scatter(ee_pos(:,1), ee_pos(:,2));
xlabel("x");
ylabel("y");
axis equal;
grid on;
title('Trajectory colored by speed');

figure(Name="FT",NumberTitle="off");

plot(t, sqrt(FT(:,1).^2 + FT(:,2).^2));
xlabel("t [s]");
ylabel("|F{xy}|");



figure(Name="c_des",NumberTitle="off");

plot(t, c_des(:,1));
xlabel("t [s]");
ylabel("|F{xy}|");