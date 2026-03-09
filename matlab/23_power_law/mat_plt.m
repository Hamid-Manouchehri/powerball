clc; clear; close all;

schunk_file = "midDamp_50_schunk.mat";  % TODO
hand_written_file = "hand_written_mat_save.mat";  % TODO
dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";
schunk_file = dir + schunk_file;
hand_written_file = dir + hand_written_file;

hand_written_drawing = load(hand_written_file);
schunk_admittance = load(schunk_file);

% hand-written (hw) data:
hw_t = hand_written_drawing.t;
hw_x = hand_written_drawing.x;
hw_y = hand_written_drawing.y;
hw_R = hand_written_drawing.R;
hw_K = hand_written_drawing.K_hat;
hw_beta = hand_written_drawing.beta_hat;
hw_window_center_time = hand_written_drawing.window_center_time;

% Schunk admittance control data:
schunk_t = schunk_admittance.schunk_time_s;
schunk_x = schunk_admittance.x;
schunk_y = schunk_admittance.y;
schunk_R = schunk_admittance.R;
schunk_K = schunk_admittance.K_hat;
schunk_beta = schunk_admittance.beta_hat;
schunk_window_center_time = schunk_admittance.window_center_time;

figure;
subplot(1,2,1);
plot(hw_x, hw_y, "r");
% xlabel("x (m)");
% ylabel("y (m)");
title("hand-written path.")
subplot(1,2,2);
plot(schunk_y, schunk_x, "r")
title("admittance controlled path.")
% figure;
% plot(hw_window_center_time, hw_beta, 'r.-');
% grid on;
% xlabel('time [s]');
% ylabel('\beta');
% title('Segment-wise power law exponent');

% figure;
% plot(hw_window_center_time, hw_K, 'r.-');
% grid on;
% xlabel('time [s]');
% ylabel('K');
% title('Segment-wise affine velocity gain');

figure;
subplot(1,2,1)
% beta_error = abs(1/3 - hw_beta);
histogram(hw_beta,-1:0.01:1,'Normalization','probability');
grid on;
% xlabel('time [s]');
ylabel('\beta hat');
title('hand written Beta');
subplot(1,2,2);
histogram(schunk_beta,-1:0.01:1,'Normalization','probability');
title('schunk admittance Beta; C=50');
grid on;

figure;
subplot(1,2,1);
plot(hw_R, "o");
title("R hand-written");
ylim([0,6]);
subplot(1,2,2);
plot(schunk_R, "o");
title("R admittance control");

figure;
subplot(1,2,1)
plot(hw_K,'o');
ylim([0 10])

figure;
subplot(1,2,1)
p_hw = histogram(hw_K/10, 0:0.001:.2, "Normalization","probability");
entropy_hw = -sum(p_hw.Values.*log2(p_hw.Values));
subplot(1,2,2)
p_schunk = histogram(schunk_K, 0:0.001:.2, "Normalization","probability");
entropy_schunk = -sum(p_schunk.Values.*log2(p_schunk.Values));