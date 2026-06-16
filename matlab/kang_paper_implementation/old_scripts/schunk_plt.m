clear; clc; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

read_schunk_file = "indirect_5_schunk.csv";  % TODO
save_mat_file = "indirect_5_schunk.mat";  % TODO
save_schunk_csv = "test.csv";  % TODO


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
rf     = schunk_table{:,44:46};  % filtered reference var
F_exp  = schunk_table{:,47:49};  % expected force
beta   = schunk_table{:,50};     % Beta
Ci     = schunk_table{:,51:59};  % virtual damping
Mi_inv = schunk_table{:,60:68};  % inv virtual mass

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

% T = array2table([t, ee_pos(:,1:2), ee_vel(:,1:2), FT(:,1:2)], ...
%     'VariableNames', {'Time', ...
%                       'X','Y', ...
%                       'Vx','Vy', ...
%                       'Fx', 'Fy'});
% writetable(T, csvDir + save_schunk_csv);

% save(matDir+save_mat_file);  % comment in case you do n't want to save the data

figure(Name="ee_pos",NumberTitle="off");


speed = sqrt(v_meas(:,1).^2 + v_meas(:,2).^2);

scatter(ee_pos(:,1), ee_pos(:,2), 20, speed, 'filled');
xlabel("x");
ylabel("y");
axis equal;
grid on;
cb = colorbar;
colormap jet;
cb.Label.String = "speed (m/s)";
title('Trajectory colored by speed');


figure(Name="T_V_F", NumberTitle="off");
subplot(2,1,1);
plot(t, v_meas(:,1));
hold on;
plot(t, v_meas(:,2));
ylabel("Velocity [m/s]");
legend(["X", "Y"]);
subplot(2,1,2);
plot(t, F_cmd(:,1));
hold on;
plot(t, F_cmd(:,2));
xlabel("Time [s]")
ylabel("F_{cmd} [N]");
legend(["X", "Y"]);

% figure(Name="ee_pos",NumberTitle="off");
% % plot(ee_pos(:,1), ee_pos(:,2));
% subplot(2,1,1);
% plot(t, ee_pos(:,1));
% % ylim([-0.2, 0.5]);
% subplot(2,1,2);
% plot(t, ee_pos(:,2));
% % ylim([-0.2, 0.5]);



% figure(Name="Qdot", NumberTitle="off");
% 
% 
% 
% subplot(2,1,1);
% plot(t, Qdot_a);
% ylabel("Qdot_a");
% legend(["Qdot_a1", "Qdot_a2", "Qdot_a3", "Qdot_a4", "Qdot_a5", "Qdot_a6"]);
% subplot(2,1,2);
% plot(t, Qdot);
% ylabel("Qdot");
% legend(["Qdot1", "Qdot2", "Qdot3", "Qdot4", "Qdot5", "Qdot6"]);



% figure(Name="FT",NumberTitle="off");
% 
% 
% 
% subplot(2,1,1);
% plot(t, FT);
% xlabel("Time [s]");
% ylabel("FT");
% legend(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]);
% subplot(2,1,2);
% plot(t, F_cmd);
% ylabel("F_{cmd}")
% xlabel("Time [s]");
% legend(["F_cmd_x", "F_cmd_y", "F_cmd_z", "F_cmd_rx", "F_cmd_ry", "F_cmd_rz"]);



% figure(Name="vel", NumberTitle="off");
% 
% 
% subplot(3,1,1)
% plot(t, v_meas);
% legend(["v_{meas_x}", "v_{meas_y}", "v_{meas_z}" ...
%        ,"v_{meas_rx}", "v_{meas_ry}", "v_{meas_rz}"]);
% xlabel("Time [s]");
% ylabel("V_{meas}");
% subplot(3,1,2)
% plot(t, v_meas_lpf);
% legend(["v_{meas_x}", "v_{meas_y}", "v_{meas_z}" ...
%        ,"v_{meas_rx}", "v_{meas_ry}", "v_{meas_rz}"]);
% xlabel("Time [s]");
% ylabel("filtered V_{meas}");
% subplot(3,1,3);
% plot(t, vel);
% xlabel("Time [s]");
% ylabel("Vel");
% legend(["Vx", "Vy", "Vz", "Vrx", "Vry", "Vrz"]);



% figure(Name="Cd",NumberTitle="off");



% subplot(2,1,1);
% plot(t, c_des(:,1), LineStyle="--");
% hold on;
% plot(t, c_des(:,2), LineStyle=":");
% legend(["c_{des}_x", "c_{des}_y"]);
% ylabel("var damping C_{des}");
% subplot(2,1,2);
% plot(t, Cd(:,1), LineStyle="--");
% hold on;
% plot(t, Cd(:,2), LineStyle=":");
% legend(["cd_y", "cd_x"]);
% ylabel("var damping C_d");
% xlabel("Time [s]");
% ylim([80 120]);


% figure(Name="diff Cd",NumberTitle="off");
% 
% plot(diff(Cd(:,1))./diff(t))



% figure(Name="V_F_D", NumberTitle="off");
% 
% 
% 
% vel_mag = sqrt(vel(:,1).^2 + vel(:,2).^2);
% F_cmd_mag = sqrt(F_cmd(:,1).^2 + F_cmd(:,2).^2);
% Cd_mag = sqrt(Cd(:,1).^2 + Cd(:,2).^2);
% F_min_x = Cd(:,1).*(-vel(:,1));
% 
% subplot(2,1,1);
% plot(-vel(:,1), Cd(:,1), "r.");
% xlabel("Vel magnitude");
% ylabel("Var C");
% xlim([0, 0.3]);
% subplot(2,1,2);
% plot(-vel(:,1), F_min_x, "r.");
% xlabel("Vel magnitude");
% ylabel("F_{min}");
% xlim([0, 0.3]);


% figure(Name="T_F_D", NumberTitle="off");
% 
% 
% subplot(2,1,1);
% plot(t, Cd(:,1), "r.");
% xlabel("Time [s]");
% ylabel("Var C");
% % xlim([0, 0.3]);
% subplot(2,1,2);
% yyaxis left
% plot(t, -F_cmd(:,1), "r.");
% xlabel("Time [s]");
% ylabel("F_{cmd}");
% yyaxis right
% plot(t,-vel(:,1));
% ylabel("Velocity (m/s)");
% xlim([0, 0.3]);