clear; clc; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

read_schunk_file = "test_2d_var_damp_schunk.csv";

dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/";

read_schunk_file = dir + read_schunk_file;
schunk_table = readtable(read_schunk_file);

t      = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
Q      = schunk_table{:,2:7};
Qdot_a = schunk_table{:,8:13};   % Schunk angular velocity
Qdot   = schunk_table{:,14:19};  % Input veloc6ity command (admittance control)
FT     = schunk_table{:,20:25};  % Raw FT
F_cmd  = schunk_table{:,26:31};  % FT after filtering
v_meas = schunk_table{:,32:37};  % end-effector velocity
vel    = schunk_table{:,38:43};  % admittance vel
Cd     = schunk_table{:,44:49};  % Variable damping

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
plot(ee_pos(:,1), ee_pos(:,2));
xlabel("x");
ylabel("y");

% figure(Name="ee_pos",NumberTitle="off");
% % plot(ee_pos(:,1), ee_pos(:,2));
% subplot(2,1,1);
% plot(t, ee_pos(:,1));
% % ylim([-0.2, 0.5]);
% subplot(2,1,2);
% plot(t, ee_pos(:,2));
% % ylim([-0.2, 0.5]);

figure(Name="Qdot", NumberTitle="off");
subplot(2,1,1);
plot(t, Qdot_a);
ylabel("Qdot_a");
legend(["Qdot_a1", "Qdot_a2", "Qdot_a3", "Qdot_a4", "Qdot_a5", "Qdot_a6"]);
subplot(2,1,2);
plot(t, Qdot);
ylabel("Qdot");
legend(["Qdot1", "Qdot2", "Qdot3", "Qdot4", "Qdot5", "Qdot6"]);

figure(Name="FT",NumberTitle="off");
subplot(2,1,1);
plot(t, FT);
ylabel("FT");
legend(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]);
subplot(2,1,2);
plot(t, F_cmd);
ylabel("F_{cmd}")
legend(["F_cmd_x", "F_cmd_y", "F_cmd_z", "F_cmd_rx", "F_cmd_ry", "F_cmd_rz"]);

figure(Name="v_meas", NumberTitle="off");
plot(t, v_meas);
legend(["v_{meas_x}", "v_{meas_y}", "v_{meas_z}" ...
       ,"v_{meas_rx}", "v_{meas_ry}", "v_{meas_rz}"]);

figure(Name="vel", NumberTitle="off");
plot(t, vel);
legend(["Vx", "Vy", "Vz", "Vrx", "Vry", "Vrz"]);

figure(Name="Cd",NumberTitle="off");
plot(t, Cd(:,1), LineStyle="--");
hold on;
plot(t, Cd(:,2), LineStyle=":");
legend(["cd_y", "cd_x"]);
ylabel("var damping");
xlabel("Time [s]");
ylim([0 120]);

figure(Name="diff Cd",NumberTitle="off");
plot(diff(Cd(:,2))./diff(t))






