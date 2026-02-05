clc; clear;

csvFile = "../data/admittance_control/test_1_020326/maze_damp_80.000000_01.csv";  % TODO

opts = detectImportOptions(csvFile, 'Delimiter', ',');
opts.VariableNamingRule = "preserve";   % keep original header text
tsCol = opts.VariableNames{contains(opts.VariableNames, "TimeStamp")};
opts = setvartype(opts, tsCol, "string");
opts = setvaropts(opts, tsCol, 'WhitespaceRule', 'trim');

T = readtable(csvFile, opts);
t = datetime(T.TimeStamp, 'InputFormat', 'HH:mm:ss:SSSSSS');
% t = seconds(t - t(1));
Q  = T{:, {'Q1','Q2','Q3','Q4','Q5','Q6'}};
dQ = T{:, {'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6'}};
FT = T{:, {'FT1','FT2','FT3','FT4','FT5','FT6'}};

figure; hold on; grid on;

% subplot(3,1,1); hold on;
% plot(t, Q(:,1)); title('Joint Positions'); ylabel('rad');

% subplot(3,1,2); hold on;
% plot(t, dQ); title('Joint Velocities'); ylabel('rad/s');

% subplot(3,1,3); hold on;
plot(t, FT(:,1:2)); title('Force/Torque'); ylabel('N / Nm'); xlabel('Time');
