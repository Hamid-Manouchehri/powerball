clear; clc; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

read_schunk_file = "test_var_damp_schunk.csv";

dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/";

read_schunk_file = dir + read_schunk_file;
schunk_table = readtable(read_schunk_file);

t = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
Q   = schunk_table{:,2:7};
Qd  = schunk_table{:,8:13};
FT  = schunk_table{:,14:19};
vel = schunk_table{:,20:25};  % admittance vel
Cd = schunk_table{:, 26:31};

schunk_numOfDataSamples = size(Q, 1);

ee_pos    = zeros(schunk_numOfDataSamples,3);
ee_orient = zeros(schunk_numOfDataSamples,3);
ee_vel    = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qd(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

figure(1);
plot(ee_pos(:,1), ee_pos(:,2));

figure(2);
% plot(ee_pos(:,1), ee_pos(:,2));
subplot(2,1,1);
plot(t, ee_pos(:,1));
% ylim([-0.2, 0.5]);
subplot(2,1,2);
plot(t, ee_pos(:,2));
% ylim([-0.2, 0.5]);

figure(3);
plot(t, FT);
legend(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]);

figure(4);
plot(t, Cd(:,1), LineStyle="--");
hold on;
plot(t, Cd(:,2), LineStyle=":");
legend(["cd1", "cd2"]);

figure(5);
plot(diff(Cd(:,2))./diff(t))

figure(6);
plot(t, vel);
legend(["Vx", "Vy", "Vz", "Vrx", "Vry", "Vrz"]);
