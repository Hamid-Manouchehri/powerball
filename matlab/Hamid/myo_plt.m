clc; clear; close all;

myo_csv = "/home/hamid-tuf/projects/PythonMyoLinux/data/myo_20260228_183437.csv";
T = readtable(myo_csv);

% --- time vector ---
% If the first column is time, extract as numeric array:
t = T{:,1};

% Optional: if it's a string timestamp, convert to seconds
% t = seconds(duration(T{:,1},'InputFormat','mm:ss.SSS'));  % adjust format if needed

% --- EMG plot (assume EMG is in columns 2..9) ---
figure;
sgtitle("EMG data");
for i = 1:8
    subplot(4,2,i);
    y = T{:, 1+i};   % columns 2..9
    plot(t, y);
    grid on;
    title(sprintf("EMG%d", i));
    xlabel("time");
    ylabel("raw");
end

% --- IMU plot (use names if available; else use your indices) ---
vars = string(T.Properties.VariableNames);

% Try to find expected columns by name
ori_idx  = find(startsWith(lower(vars), "ori"));
acc_idx  = find(startsWith(lower(vars), "acc"));
gyr_idx  = find(startsWith(lower(vars), "gyr"));

% Fallback to your ranges if names not found
if numel(ori_idx) ~= 4
    ori_idx = 2:5;   % CHANGE if your CSV differs
end
if numel(acc_idx) ~= 3
    acc_idx = 6:8;   % CHANGE if your CSV differs
end
if numel(gyr_idx) ~= 3
    gyr_idx = 9:11;  % CHANGE if your CSV differs
end

figure;
sgtitle("IMU data");

subplot(3,1,1);
plot(t, T{:, ori_idx});
grid on; ylabel("orientation (quat)");
legend(vars(ori_idx), "Interpreter","none", "Location","best");

subplot(3,1,2);
plot(t, T{:, gyr_idx});
grid on; ylabel("gyro");
legend(vars(gyr_idx), "Interpreter","none", "Location","best");

subplot(3,1,3);
plot(t, T{:, acc_idx});
grid on; ylabel("accel");
legend(vars(acc_idx), "Interpreter","none", "Location","best");
xlabel("time");