clc; clear; close all;


schunk_csv = "/home/hamid-tuf/projects/powerball/data/admittance/test_damp_50_01.csv";
emg_csv = "/home/hamid-tuf/projects/powerball/data/admittance/test_myo.csv";

schunk_table = readtable(schunk_csv);
schunk_time_s = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
emg_table = readtable(emg_csv);
emg_time_s = (emg_table.Time_us - emg_table.Time_us(1)) / 1e6;


figure;
sgtitle("EMG data");
for i=1:8

    subplot(4,2,i);
    field_name = sprintf('EMG%d', i);
    emg_data = emg_table.(field_name);
    plot(emg_time_s, emg_data);
    legend;

end


orient_eul = zeros(size(emg_table,1), 3);
ang_vel = zeros(size(emg_table,1), 3);
lin_accel = zeros(size(emg_table,1), 3);

for i=1:size(emg_table,1)

    orient_eul(i,:) = quat_to_euler_fcn([emg_table.ORI1(i), emg_table.ORI2(i), ...
                                         emg_table.ORI3(i), emg_table.ORI4(i)]);
    ang_vel(i,:) = [emg_table.GYR1(i), emg_table.GYR2(i), emg_table.GYR3(i)];
    lin_accel(i,:) = [emg_table.ACC1(i), emg_table.ACC2(i), emg_table.ACC3(i)];

end

figure;
sgtitle("IMU data");
subplot(3,1,1);
plot(emg_time_s, orient_eul); ylabel("orientation");
subplot(3,1,2);
plot(emg_time_s, ang_vel); ylabel("ang vel");
subplot(3,1,3);
plot(emg_time_s, lin_accel); ylabel("lin accel");


figure;
sgtitle("Schunk Data");
subplot(4,1,1);
plot(schunk_time_s, schunk_table{:,2:7}); ylabel("Q (rad)"); 
legend(["Q1", "Q2", "Q3", "Q4", "Q5", "Q6"]);
subplot(4,1,2);
plot(schunk_time_s, schunk_table{:,8:13}); ylabel("ang vel (rad/s)");
legend(["dQ1", "dQ2", "dQ3", "dQ4", "dQ5", "dQ6"]);
subplot(4,1,3);
plot(schunk_time_s, schunk_table{:,14:16}); ylabel("Force (N)");
legend(["Fx", "Fy", "Fz"]);
subplot(4,1,4);
plot(schunk_time_s, schunk_table{:,20:22}); ylabel("tool vel (m/s)");
legend(["Vx", "Vy", "Vz"]);