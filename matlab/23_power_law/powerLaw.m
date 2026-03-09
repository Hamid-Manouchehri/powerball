clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/");

schunk_mat_file_save = "midDamp_50_schunk.mat";
schunk_csv = "midDamp_damp_50_schunk.csv";  % TODO
myo_csv = "midDamp_myo.csv";  % TODO

mat_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";
csv_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/";
schunk_csv = csv_dir + schunk_csv;
myo_csv = csv_dir + myo_csv;

schunk_table = readtable(schunk_csv);
schunk_time_s = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
myo_table = readtable(myo_csv);
myo_time_s = (myo_table.Time_us - myo_table.Time_us(1)) / 1e6;

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

x = ee_pos(:,1);
y = ee_pos(:,2);

xDot = ee_vel(:, 1);
yDot = ee_vel(:, 2);

xDDot = zeros(schunk_numOfDataSamples, 1);
yDDot = zeros(schunk_numOfDataSamples, 1);

for i = 2:schunk_numOfDataSamples-1
    dt = schunk_time_s(i+1) - schunk_time_s(i-1);
    xDDot(i) = (xDot(i+1) - xDot(i-1)) / dt;
    yDDot(i) = (yDot(i+1) - yDot(i-1)) / dt;
end

R     = zeros(schunk_numOfDataSamples, 1);  % radius of curvature
vNorm = zeros(schunk_numOfDataSamples, 1);
alpha = 0.;  % TODO

% Calculating radius of curvature:
eps_v = 1e-6;
eps_c = 1e-12;   % to avoid division by zero in curvature term

% Calculating the radius of segment-wise curvature
for i = 2:schunk_numOfDataSamples-1
    vNorm(i) = norm([xDot(i) yDot(i)]);
    denom = abs(xDot(i)*yDDot(i) - yDot(i)*xDDot(i));

    if vNorm(i) > eps_v && denom > eps_c
        R(i) = vNorm(i)^3 / denom;
        R(i) = R(i) / (1 + alpha*R(i));
    else
        R(i) = NaN;
    end
end

windowSize = 10;   % TODO
overlap = 0;       % TODO
step = windowSize - overlap;

num_windows = floor((schunk_numOfDataSamples - windowSize) / step) + 1;

beta_hat = NaN(num_windows, 1);
K_hat = NaN(num_windows, 1);
window_center_idx = NaN(num_windows, 1);

% Fit a linear regression model: polyfit(x,y,1):
j = 1;
for i = 1:step:schunk_numOfDataSamples-windowSize+1

    idx1 = i;
    idx2 = i + windowSize - 1;

    vNorm_window = vNorm(idx1:idx2);
    R_window = R(idx1:idx2);

    valid_window = ~isnan(R_window) & ~isnan(vNorm_window) & (R_window > 0) & (vNorm_window > 0);

    if sum(valid_window) >= 2
        p = polyfit(log(R_window(valid_window)), log(vNorm_window(valid_window)), 1);
        beta_hat(j) = p(1);
        K_hat(j) = exp(p(2));
    end

    window_center_idx(j) = floor((idx1 + idx2) / 2);
    j = j + 1;
end

window_center_time = schunk_time_s(window_center_idx);

figure;
plot(ee_pos(:, 2), ee_pos(:, 1), "r");
xlabel("x (m)");
ylabel("y (m)");
title("admittance controlled maze path.")

figure;
plot(window_center_time, beta_hat, 'r.-');
grid on;
xlabel('time [s]');
ylabel('\beta');
title('Segment-wise power law exponent');

figure;
plot(window_center_time, K_hat, 'r.-');
grid on;
xlabel('time [s]');
ylabel('K');
title('Segment-wise affine velocity gain');

figure;
beta_error = abs(1/3 - beta_hat);
% plot(window_center_time, beta_error, "o");
histogram(beta_hat,-1:0.01:1,'Normalization','probability');
grid on;
% xlabel('time [s]');
ylabel('\beta hat');
title('Beta error');

figure;
plot(R, "o")
title("Radius of curvature");

figure;
subplot(2,1,1)
plot(schunk_time_s, ee_pos(:,1), 'o');
ylabel("x");
subplot(2,1,2)
plot(schunk_time_s, ee_pos(:,2), 'r.');
ylabel("y");
% disp(["beta_hat =", num2str(beta_hat)])
% disp(["k_hat =", num2str(K_hat)]);
% disp(["beta_error =", num2str(beta_error)]);

% figure; plot(log(k(valid)), log(omega(valid)), '.'); grid on;
% xlabel("log(kappa)"); ylabel("log(omega)");
% title("2/3 power law fit (slope = beta)");

save((mat_dir+schunk_mat_file_save), 'schunk_time_s', 'x', 'y', ...
                                     'window_center_time', 'beta_hat', ...
                                     'K_hat', ...
                                     'R');

