clc; clear; close all;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp100 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];

dataset   = fcn_load_csv(fName_damp100);
t         = dataset.timesteps;
Q         = dataset.q;
dQ        = dataset.qd;
FT        = dataset.FT;

numDataSamples = size(Q,1);

% ---------- sampling rate ----------
dt_vec = diff(t);
dt_med = median(dt_vec);
Fs = 1/dt_med;                 % (better than floor(N/t(end)) if dt not exact)
L  = numDataSamples;

% ---------- pick force signal (1D) ----------
% Option 1: use Fx only (matches your current plotting)
f_sig = FT(:,1);

% Option 2 (uncomment): magnitude of force vector
% f_sig = vecnorm(FT(:,1:3),2,2);

% ---------- HSO / Is parameters ----------
Tw      = 0.50;                 % window length [s] (0.25–0.5s typical)
Nw      = max(64, 2^nextpow2(round(Tw*Fs)));   % window samples (power of 2)
Hop     = max(1, round(0.02*Fs));              % update every 20 ms
fc      = 3.0;                  % crossover frequency [Hz] (your omegaC=2*pi*3)
lambda  = 0.95;                 % forgetting factor
fmax    = max(abs(f_sig));      % normalize RMS (better: choose expected max force)

% omega0/omegaC (rad/s)
omega0 = 2*pi*(Fs/Nw);          % first non-DC bin rad/s
omegaC = 2*pi*fc;

% ---------- compute sliding HSO Iv and improved Is ----------
w = hann(Nw);                   % window to reduce leakage (recommended)

% time indices where we can compute a full window
k0 = Nw;
k_list = k0:Hop:L;

Iv = nan(size(k_list));
Is = nan(size(k_list));
Ifrms = nan(size(k_list));

Is_prev = 0;

for ii = 1:numel(k_list)
    k = k_list(ii);

    xw = f_sig(k-Nw+1:k);
    xw = xw - mean(xw);         % remove DC (important)
    xw = xw .* w;

    X = fft(xw);
    P = abs(X)/Nw;              % amplitude spectrum (consistent with your P2=abs(X)/L)
    P1 = P(1:floor(Nw/2)+1);    % one-sided

    f_bins = (0:floor(Nw/2))*(Fs/Nw);

    % exclude DC bin in denominator (start at bin 2 in MATLAB indexing)
    idx_total = 2:numel(P1);    % f > 0
    idx_hf    = find(f_bins >= fc, 1, 'first'):numel(P1);

    denom = sum(P1(idx_total)) + eps;
    numer = sum(P1(idx_hf)) + eps;

    Iv(ii) = numer/denom;       % Eq(4) HSO

    % rmsf = sqrt(mean((xw).^2));
    % Ifrms(ii) = min(max(rmsf/(fmax+eps), 0), 1);  % Eq(6), clipped [0,1]
    % 
    % Is_curr = Iv(ii)*Ifrms(ii) + lambda*Is_prev;  % Eq(5)
    % Is(ii) = Is_curr;
    % Is_prev = Is_curr;
end

t_idx = t(k_list);

% ---------- plots ----------
figure;

subplot(3,1,1);
plot(t, f_sig, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('Force (N)');
title('Force signal used for instability index');

subplot(3,1,2);
plot(t_idx, Iv, 'LineWidth', 1);
grid on; xlabel('time (s)'); ylabel('I_v');
title(sprintf('HSO index I_v (fc = %.1f Hz)', fc));
ylim([0 1]);

% subplot(3,1,3);
% plot(t_idx, Is, 'LineWidth', 1);
% grid on; xlabel('time (s)'); ylabel('I_s');
% title(sprintf('Improved instability index I_s (\\lambda=%.2f)', lambda));

fprintf('Fs = %.3f Hz, Nw = %d samples, Tw = %.3f s\n', Fs, Nw, Nw/Fs);
fprintf('omega0 = %.3f rad/s, omegaC = %.3f rad/s\n', omega0, omegaC);
