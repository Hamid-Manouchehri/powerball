clc; clear; close all;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp10 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];
fName_damp100 = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];

dataset = fcn_load_csv(fName_damp10);
t = dataset.timesteps;
Q = dataset.q;
dQ = dataset.qd;
FT = dataset.FT;

numDataSamples = size(Q,1);

dt_vec = diff(t);
dt_med = median(dt_vec);
Fs = 1/dt_med;  % sampling freq
L  = numDataSamples;

F_x = FT(:,1);

% HSO parameters:
Tw = 0.5;  % window length [s] 
Nw = round(Tw*Fs);  % num samples per windows
fft_step = round(0.02*Fs);  % windows step-size; update every 20 ms
% lambda = 0.95;  % TODO; forgetting factor for Is
% F_x_max = max(abs(F_x));  % normalize RMS (better: choose expected max force)

fc = 3.0;  % TODO
omega0 = 2*pi*(Fs/Nw);  % [rad/s]
omegaC = 2*pi*fc;

w = hann(Nw);  % window to reduce leakage (recommended)

k_list = Nw:fft_step:L;  % sliced samples for fft

Iv = zeros(numel(k_list),1);
Is = zeros(numel(k_list),1);
Ifrms = zeros(numel(k_list),1);

Is_prev = 0;

for ii = 1:numel(k_list)
    k = k_list(ii);

    xw = F_x(k-Nw+1:k);
    xw = xw - mean(xw); % removing DC
    xw = xw .* w;

    X = fft(xw);
    P = abs(X)/Nw;
    P1 = P(1:floor(Nw/2)+1);  % one-sided fft

    f_bins = (0:floor(Nw/2))*(Fs/Nw);

    idx_total = 2:numel(P1);  % f > 0
    idx_hf = find(f_bins >= fc, 1):numel(P1);

    numer = sum(P1(idx_hf));
    denom = sum(P1(idx_total));

    Iv(ii) = numer/denom;  % Eq(4) HSO

    % rmsf = sqrt(mean((xw).^2));
    % Ifrms(ii) = min(max(rmsf/(F_x_max+eps), 0), 1);  % clipped [0,1]
    % 
    % Is_curr = Iv(ii)*Ifrms(ii) + lambda*Is_prev;  %
    % Is(ii) = Is_curr;
    % Is_prev = Is_curr;
end

t_idx = t(k_list);

% plots
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
plot(t_idx, Iv, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('HSO (I_v)');
% title(sprintf('HSO index I_v (fc = %.1f Hz)', fc));
ylim([0 1]);

% subplot(4,1,4);
% plot(t_idx, Is, 'LineWidth', 1);
% grid on; xlabel('time (s)'); ylabel('I_s');
% % title(sprintf('Improved instability index I_s (\\lambda=%.2f)', lambda));

fprintf('Fs = %.3f Hz, Nw = %d samples, Tw = %.3f s\n', Fs, Nw, Nw/Fs);
fprintf('omega0 = %.3f rad/s, omegaC = %.3f rad/s\n', omega0, omegaC);

