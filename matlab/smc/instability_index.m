clc; clear; close all;

% User Settings
% - One dataset is analyzed at a time.
% - Moving-window length: 2 seconds.
% - Window overlap: none.
% - Instability index formula:
%       I_v = sum(|FFT force| for f >= fc) / sum(|FFT force| for f > 0)
% - Force units: N, from FT(:, forceAxis).
% - Time units: seconds.

dataFolder = fullfile('data', 'schunk_data');
datasetFile = fullfile(dataFolder, 'S2_var_clover_schunk.csv');

forceAxis = 1;  % 1 = Fx, 2 = Fy, 3 = Fz, 4 = Tx, 5 = Ty, 6 = Tz

dataset = readtable(datasetFile);

t = (dataset.Time_us - dataset.Time_us(1)) / 1e6;
FT = dataset{:, 20:25};  % raw force/torque from Schunk CSV

numDataSamples = size(FT, 1);
dt_vec = diff(t);
dt_med = median(dt_vec);
Fs = 1/dt_med;  % sampling freq
L  = numDataSamples;

F_x = FT(:, forceAxis);

% HSO parameters:
Tw = 2.0;  % moving-window length [s]
Nw = round(Tw*Fs);  % samples per window
windowStep = Nw;  % no overlap: next window starts after current window ends
% lambda = 0.95;  % TODO; forgetting factor for Is
% F_x_max = max(abs(F_x));  % normalize RMS (better: choose expected max force)

fc = 3.0;  % TODO
omega0 = 2*pi*(Fs/Nw);  % [rad/s]
omegaC = 2*pi*fc;

w = hann(Nw);  % window to reduce leakage (recommended)

windowStart = 1:windowStep:(L - Nw + 1);
windowEnd = windowStart + Nw - 1;
windowCenter = round((windowStart + windowEnd)/2);

Iv = zeros(numel(windowStart),1);
Is = zeros(numel(windowStart),1);
Ifrms = zeros(numel(windowStart),1);

Is_prev = 0;

for ii = 1:numel(windowStart)
    idx = windowStart(ii):windowEnd(ii);

    xw = F_x(idx);
    xw = xw - mean(xw); % removing DC
    xw = xw .* w;

    X = fft(xw);
    P = abs(X)/Nw;
    P1 = P(1:floor(Nw/2)+1);  % one-sided fft

    f_bins = (0:floor(Nw/2))*(Fs/Nw);

    idx_total = 2:numel(P1);  % f > 0
    first_hf_idx = find(f_bins >= fc, 1);

    if isempty(first_hf_idx)
        numer = 0;
    else
        idx_hf = first_hf_idx:numel(P1);
        numer = sum(P1(idx_hf));
    end

    denom = sum(P1(idx_total));

    Iv(ii) = numer/(denom + eps);  % Eq(4) HSO

    % rmsf = sqrt(mean((xw).^2));
    % Ifrms(ii) = min(max(rmsf/(F_x_max+eps), 0), 1);  % clipped [0,1]
    % 
    % Is_curr = Iv(ii)*Ifrms(ii) + lambda*Is_prev;  %
    % Is(ii) = Is_curr;
    % Is_prev = Is_curr;
end

t_idx = t(windowCenter);
windowStartTime = t(windowStart);
windowEndTime = t(windowEnd);

instabilityTable = table(windowStartTime, windowEndTime, t_idx, Iv, ...
    'VariableNames', {'window_start_s', 'window_end_s', 'window_center_s', 'Iv'});

% plots
[ee_pos, ~, ~] = schunk_ee_pos_fun(datasetFile);

figure(Name="ee_pos", NumberTitle="off");

plot(ee_pos(:,1), ee_pos(:,2))


figure;

subplot(3,1,1);
plot(t, F_x, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('Force (N)');
% title('Force signal used for instability index');

% fft_F_x = fft(F_x);
% P11 = abs(fft_F_x)/L;
% P12 = P11(1:floor(L/2)+1);
% subplot(4,1,2);
% semilogx(Fs/L*(0:(L/2)), P12, 'LineWidth', 1);
% grid on; xlabel('f (Hz)'); ylabel('power (dB)');
X = fft(F_x);
% one-sided spectrum
P2 = abs(X)/L;
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:floor(L/2))/L;
mag_dB = 20*log10(P1 + eps);  % eps avoids log(0)
subplot(3,1,2);
semilogx(f, mag_dB);
grid on; xlabel("f (Hz)"); ylabel("power (dB)");

subplot(3,1,3);
stairs(t_idx, Iv, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('HSO (I_v)');
% title(sprintf('HSO index I_v (fc = %.1f Hz)', fc));
ylim([0 1]);

% subplot(4,1,4);
% plot(t_idx, Is, 'LineWidth', 1);
% grid on; xlabel('time (s)'); ylabel('I_s');
% % title(sprintf('Improved instability index I_s (\\lambda=%.2f)', lambda));

fprintf('Fs = %.3f Hz, Nw = %d samples, Tw = %.3f s\n', Fs, Nw, Nw/Fs);
fprintf('Window step = %d samples, overlap = 0 samples\n', windowStep);
fprintf('Number of complete windows = %d\n', numel(windowStart));
fprintf('omega0 = %.3f rad/s, omegaC = %.3f rad/s\n', omega0, omegaC);
disp(instabilityTable);
