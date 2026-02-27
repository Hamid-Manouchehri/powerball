clc; clear; close all;

emg_csv = "../../data/myo/emg_rec1.csv";
imu_csv = "../../data/myo/imu_rec1.csv";

emg_table = readtable(emg_csv);
emg_time_s = (emg_table.Time(:) - emg_table.Time(1)) / 1000;
imu_table = readtable(imu_csv);
imu_time_s = (imu_table.Time(:) - imu_table.Time(1)) / 1000;


figure;
sgtitle("EMG data");
for i=1:8

    subplot(4,2,i);
    field_name = sprintf('EMG%d', i);
    emg_data = emg_table.(field_name);
    plot(emg_time_s, emg_data);
    legend;

end


orient_eul = zeros(size(imu_table,1), 3);
ang_vel = zeros(size(imu_table,1), 3);
lin_accel = zeros(size(imu_table,1), 3);

for i=1:size(imu_table,1)

    orient_eul(i,:) = quat_to_euler_fcn([imu_table.ORI1(i), imu_table.ORI2(i), ...
                                         imu_table.ORI3(i), imu_table.ORI4(i)]);
    ang_vel(i,:) = [imu_table.GYR1(i), imu_table.GYR2(i), imu_table.GYR3(i)];
    lin_accel(i,:) = [imu_table.ACC1(i), imu_table.ACC2(i), imu_table.ACC3(i)];

end

figure;
sgtitle("IMU data");
subplot(3,1,1);
plot(imu_time_s, orient_eul); ylabel("orientation");
subplot(3,1,2);
plot(imu_time_s, ang_vel); ylabel("ang vel");
subplot(3,1,3);
plot(imu_time_s, lin_accel); ylabel("lin accel");
