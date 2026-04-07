clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

read_schunk_file = "test_indirect_schunk.csv";  % TODO
save_mat_file = "test_indirect_schunk.mat";  % TODO

csvDir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/";
matDir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/mat/";

read_schunk_file = csvDir + read_schunk_file;
schunk_table = readtable(read_schunk_file);

t      = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
Q      = schunk_table{:,2:7};
Qdot_a = schunk_table{:,8:13};   % Schunk angular velocity
Qdot   = schunk_table{:,14:19};  % Input velocity command (admittance control)
FT     = schunk_table{:,20:25};  % Raw FT
F_cmd  = schunk_table{:,26:31};  % FT after filtering
v_meas = schunk_table{:,32:37};  % end-effector velocity
vel    = schunk_table{:,38:43};  % admittance vel
rf     = schunk_table{:,44:46};
F_exp  = schunk_table{:,47:49};
beta   = schunk_table{:,50};
Ci     = schunk_table{:,51:59};
Mi_inv = schunk_table{:,60:68};

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

% save(matDir+save_mat_file, "t", "ee_pos", "ee_vel", "FT", "F_cmd", ...
%     "v_meas", "vel", "c_des", "Cd");  % comment in case you do n't want to save the data


figure(Name="ee_pos",NumberTitle="off");


plot(ee_pos(:,1), ee_pos(:,2));
xlabel("x");
ylabel("y");
xlim([0.0 0.5]);
xlim([0.0 0.5]);


figure(Name="FT",NumberTitle="off");



subplot(2,1,1);
plot(t, F_cmd);
ylabel("F_{cmd}")
xlabel("Time [s]");
legend(["F_cmd_x", "F_cmd_y", "F_cmd_z", "F_cmd_rx", "F_cmd_ry", "F_cmd_rz"]);
subplot(2,1,2);
plot(t, F_exp);
xlabel("Time [s]");
ylabel("F_{exp}");
legend(["F_exp_x", "F_exp_y", "F_exp_z"]);



figure(Name="");
