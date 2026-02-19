clc; clear; close all

fs = 100;  % sampling freq (Hz)
T = 1/fs;  % sampling period
L = 1000;  % number of samples
f = fs/L*(0:L-1);

t = linspace(0,10,L);

f_sin = 1;  % Hz
f_cos = 2;  % Hz
signal = 2 + sin(2*pi*f_sin*t)+ cos(2*pi*f_cos*t);

subplot(2,1,1);
plot(t, signal); xlabel("t"); ylabel("signal");

fft_signal = fft(signal);
subplot(2,1,2);
plot(f,fft_signal)


