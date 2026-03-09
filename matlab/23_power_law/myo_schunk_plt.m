clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/");

schunk_csv = "maze_var_damp_schunk.csv";  % TODO
myo_csv = "midDamp_myo.csv";  % TODO

dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/";
schunk_csv = dir + schunk_csv;
myo_csv = dir + myo_csv;

schunk_table = readtable(schunk_csv);
schunk_time_s = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
myo_table = readtable(myo_csv);
myo_time_s = (myo_table.Time_us - myo_table.Time_us(1)) / 1e6;

Q   = schunk_table{:,2:7}; 
Qd  = schunk_table{:,8:13}; 
FT  = schunk_table{:,14:19};
vel = schunk_table{:,20:25};  % end-effector
Cd  = schunk_table{:,26:31};
schunk_numOfDataSamples = size(Q, 1);

myo_EMG    = myo_table{:,2:9}; 
myo_orient = myo_table{:,10:13};  % quaternion (w, q1, q2, q3)
myo_accel  = myo_table{:,14:16};
myo_gyr    = myo_table{:,17:19};
myo_numOfDataSamples = size(myo_EMG, 1);

% plotting EMG:
% figure;
% sgtitle("EMG data");
% for i=1:8
%     subplot(4,2,i);
%     plot(myo_time_s, myo_EMG(:,i));
% end

myo_orient_eul = zeros(size(myo_table,1), 3);
myo_ang_vel = zeros(size(myo_table,1), 3);
myo_lin_accel = zeros(size(myo_table,1), 3);

% Extracting myo data and converting to euler:
for i=1:size(myo_table,1)
    myo_orient_eul(i,:) = quat_to_euler_fun([myo_orient(i, 1), myo_orient(i, 2), ...
                                             myo_orient(i, 3), myo_orient(i, 4)]);
    myo_ang_vel(i,:) = [myo_gyr(i, 1), myo_gyr(i, 2), myo_gyr(i, 3)];
    myo_lin_accel(i,:) = [myo_accel(i, 1), myo_accel(i, 2), myo_accel(i, 3)];
end

% figure;
% sgtitle("IMU data");
% subplot(3,1,1);
% plot(myo_time_s, myo_orient_eul); ylabel("orientation (rad)");
% subplot(3,1,2);
% plot(myo_time_s, myo_ang_vel); ylabel("ang vel (rad/s)");
% subplot(3,1,3);
% plot(myo_time_s, myo_lin_accel); ylabel("lin accel (m/s^2)");


figure;
sgtitle("Schunk Data");
subplot(4,1,1);
plot(schunk_time_s, Q); ylabel("Q (rad)"); 
legend(["Q1", "Q2", "Q3", "Q4", "Q5", "Q6"]);
subplot(4,1,2);
plot(schunk_time_s, Qd); ylabel("Qd (rad/s)");
legend(["Qd1", "Qd2", "Qd3", "Qd4", "Qd5", "Qd6"]);
subplot(4,1,3);
plot(schunk_time_s, FT(:,1:3)); ylabel("Force (N)");
legend(["Fx", "Fy", "Fz"]);
subplot(4,1,4);
plot(schunk_time_s, vel(:,1:3)); ylabel("tool vel (m/s)");
legend(["Vx", "Vy", "Vz"]);

ee_pos    = zeros(schunk_numOfDataSamples,3);
ee_orient = zeros(schunk_numOfDataSamples,3);
ee_vel    = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qd(i, :)*transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

figure;
plot(ee_pos(:,2), ee_pos(:,1), 'b.');
title("Maze path");

figure;
plot(Cd);
title("Virtual Damping");
legend;
