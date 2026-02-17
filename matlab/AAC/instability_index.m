clc;        clear;         close all;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp10 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];  % TODO
fName_damp100 = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO

dataset   = fcn_load_csv(fName_damp100);
t         = dataset.timesteps; 
Q         = dataset.q; 
dQ        = dataset.qd; 
FT        = dataset.FT;
admit_vel = dataset.vel;  % admittance control velocity; cartesian space

%%% change to cartesian-space
numDataSamples = size(Q, 1);
ee_pos         = zeros(numDataSamples,3);
ee_rot         = zeros(numDataSamples,3);
ee_vel         = zeros(numDataSamples,6);
ee_accel       = zeros(numDataSamples,6);
admit_accel    = zeros(numDataSamples,6);
FTdot          = zeros(numDataSamples,3);
theta_F        = zeros(numDataSamples,1);

for i = 1:numDataSamples
    ee_vel(i, :) = dQ(i, :)*transpose(fcn_Jacob_schunk(Q(i, :)));
    T = fcn_FK_schunk(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);  % measured end-effector velocity
end

dt = diff(t);
M_admit = 0.3*diag([1, 1, 1]);
C_admit = 10*diag([1.2, 1.0, 1.0]);

figure;
subplot(3,1,1);
plot(t, FT(:,1));
xlabel("time (s)"); ylabel("F_x (N)");

Fs = floor(numDataSamples / t(end));  % sampling freq
T = 1 / Fs;  % sampling period
L = numDataSamples;  % Length of signal

subplot(3,1,2);
FTx_fft = fft(FT(:,1));

P2 = abs(FTx_fft/L);
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs/L*(0:floor(L/2));  % freq

plot(f, P1,"-.","LineWidth",1);
xlabel("f (Hz)"); ylabel("fft amplitude");
xlim([0 20])
ylim([0 3])

x = FT(:,1) - mean(FT(:,1));
X = fft(x);
f = (0:L-1)*(Fs/L);

% one-sided amplitude spectrum
P2 = abs(X)/L;
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f1 = f(1:floor(L/2)+1);

mag_dB = 20*log10(P1 + eps);  % eps avoids -Inf

subplot(3,1,3);
semilogx(f1, mag_dB); grid on;
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');

omega0 = 0.01;
omegaC = 2*pi*3;  % cut-off frequency (rad/s)

% for i = 1: