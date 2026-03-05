clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/Hamid/functions/");

schunk_csv = "highDamp_damp_100_schunk.csv";  % TODO
myo_csv = "highDamp_myo.csv";  % TODO

dir = "/home/hamid-tuf/projects/powerball/data/admittance/";
schunk_csv = dir + schunk_csv;
myo_csv = dir + myo_csv;

schunk_table = readtable(schunk_csv);
schunk_time_s = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
myo_table = readtable(myo_csv);
myo_time_s = (myo_table.Time_us - myo_table.Time_us(1)) / 1e6;

t   = schunk_table{:,1}; 
Q   = schunk_table{:,2:7}; 
Qd  = schunk_table{:,8:13}; 
FT  = schunk_table{:,14:19};
vel = schunk_table{:,20:25};  % end-effector
schunk_numOfDataSamples = size(Q, 1);

myo_EMG    = myo_table{:,2:9}; 
myo_orient = myo_table{:,10:13};  % quaternion (w, q1, q2, q3)
myo_accel  = myo_table{:,14:16};
myo_gyr    = myo_table{:,17:19};
myo_numOfDataSamples = size(myo_EMG, 1);

ee_pos   = zeros(schunk_numOfDataSamples,3);
ee_rot   = zeros(schunk_numOfDataSamples,3);
ee_vel   = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qd(i, :)*transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

% figure;
% title("Maze path");
% plot(ee_pos(:,2), ee_pos(:,1), 'b.');

x = ee_pos(:,1);
y = ee_pos(:,2);
xDot = zeros(schunk_numOfDataSamples, 1);
xDDot = zeros(schunk_numOfDataSamples, 1);
yDot = zeros(schunk_numOfDataSamples, 1);
yDDot = zeros(schunk_numOfDataSamples, 1);
k = zeros(schunk_numOfDataSamples, 1);  % curvature
omega = zeros(schunk_numOfDataSamples, 1);  % angular speed
beta = zeros(schunk_numOfDataSamples, 1);  % power law
beta_error = zeros(schunk_numOfDataSamples, 1);
K = 1;  % 2/3 power law gain

for i=2:schunk_numOfDataSamples-1

    dt = t(i+1) - t(i-1);
    xDot(i) = (x(i+1) - x(i-1)) / dt;
    yDot(i) = (y(i+1) - y(i-1)) / dt;

    xDDot(i) = (xDot(i+1) - xDot(i-1)) / dt;
    yDDot(i) = (yDot(i+1) - yDot(i-1)) / dt;

    k(i) = abs((xDot(i)*yDDot(i) - yDot(i)*xDDot(i))) / norm([xDot(i) yDot(i)])^3;
    omega(i) = k(i)*norm([xDot(i) yDot(i)]);

    beta(i) = log(omega(i) - log(K)) / log(k(i));

    beta_error(i) = 2/3 - beta(i);

end

figure;
plot(beta_error);