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
Tw = 0.50;  % window length [s] 
Nw = round(Tw*Fs);  % num samples per windows
Hop = max(1, round(0.02*Fs));  % update every 20 ms
lambda = 0.95;  % forgetting factor
fmax = max(abs(F_x));  % normalize RMS (better: choose expected max force)

fc = 3.0;
omega0 = 2*pi*(Fs/Nw);  % [rad/s]
omegaC = 2*pi*fc;

w = hann(Nw);  % window to reduce leakage (recommended)

% time indices where we can compute a full window
k0 = Nw;
k_list = k0:Hop:L;

Iv = nan(size(k_list));
Is = nan(size(k_list));
Ifrms = nan(size(k_list));

Is_prev = 0;

for ii = 1:numel(k_list)
    k = k_list(ii);

    xw = F_x(k-Nw+1:k);
    xw = xw - mean(xw); % remove DC (important)
    xw = xw .* w;

    X = fft(xw);
    P = abs(X)/Nw;  % amplitude spectrum (consistent with your P2=abs(X)/L)
    P1 = P(1:floor(Nw/2)+1);  % one-sided

    f_bins = (0:floor(Nw/2))*(Fs/Nw);

    % exclude DC bin in denominator (start at bin 2 in MATLAB indexing)
    idx_total = 2:numel(P1);  % f > 0
    idx_hf    = find(f_bins >= fc, 1, 'first'):numel(P1);

    denom = sum(P1(idx_total)) + eps;
    numer = sum(P1(idx_hf)) + eps;

    Iv(ii) = numer/denom; % Eq(4) HSO

    rmsf = sqrt(mean((xw).^2));
    Ifrms(ii) = min(max(rmsf/(fmax+eps), 0), 1);  % Eq(6), clipped [0,1]

    Is_curr = Iv(ii)*Ifrms(ii) + lambda*Is_prev;  % Eq(5)
    Is(ii) = Is_curr;
    Is_prev = Is_curr;
end

t_idx = t(k_list);

% plots
figure;

subplot(3,1,1);
plot(t, F_x, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('Force (N)');
title('Force signal used for instability index');

subplot(3,1,2);
plot(t_idx, Iv, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('I_v');
title(sprintf('HSO index I_v (fc = %.1f Hz)', fc));
ylim([0 1]);

subplot(3,1,3);
plot(t_idx, Is, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('I_s');
title(sprintf('Improved instability index I_s (\\lambda=%.2f)', lambda));

fprintf('Fs = %.3f Hz, Nw = %d samples, Tw = %.3f s\n', Fs, Nw, Nw/Fs);
fprintf('omega0 = %.3f rad/s, omegaC = %.3f rad/s\n', omega0, omegaC);
