clc; clear; close all;

schunk_file = "midDamp_50_schunk.mat";  % TODO
hand_written_file = "hand_written_mat_save.mat";  % TODO
dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";
schunk_file = dir + schunk_file;
hand_written_file = dir + hand_written_file;

hand_written_drawing = load(hand_written_file);
schunk_admittance = load(schunk_file);

% hand-written (hw) data:
t_hw = hand_written_drawing.t;
x_hw = hand_written_drawing.x;
y_hw = hand_written_drawing.y;
R_hw = hand_written_drawing.R;
K_hw = hand_written_drawing.K_hat;
beta_hw = hand_written_drawing.beta_hat;
window_center_time_hw = hand_written_drawing.window_center_time;

% Schunk admittance control data:
t_schunk = schunk_admittance.schunk_time_s;
x_schunk = schunk_admittance.x;
y_schunk = schunk_admittance.y;
R_schunk = schunk_admittance.R;
K_schunk = schunk_admittance.K_hat;
beta_schunk = schunk_admittance.beta_hat;
window_center_time_schunk = schunk_admittance.window_center_time;

figure;
subplot(1,2,1);
plot(x_hw, y_hw, "r");
% xlabel("x (m)");
% ylabel("y (m)");
title("hand-written path.")
subplot(1,2,2);
plot(y_schunk, x_schunk, "r")
title("admittance controlled path.")
% figure;
% plot(window_center_time_hw, beta_hw, 'r.-');
% grid on;
% xlabel('time [s]');
% ylabel('\beta');
% title('Segment-wise power law exponent');

% figure;
% plot(window_center_time_hw, K_hw, 'r.-');
% grid on;
% xlabel('time [s]');
% ylabel('K');
% title('Segment-wise affine velocity gain');

figure;
subplot(1,2,1)
% beta_error = abs(1/3 - hw_beta);
histogram(beta_hw,-1:0.01:1,'Normalization','probability');
grid on;
% xlabel('time [s]');
ylabel('\beta hat');
title('hand written Beta');
subplot(1,2,2);
histogram(beta_schunk,-1:0.01:1,'Normalization','probability');
title('schunk admittance Beta; C=50');
grid on;

figure;
subplot(1,2,1);
plot(R_hw, "o");
title("R hand-written");
ylim([0,6]);
subplot(1,2,2);
plot(R_schunk, "o");
title("R admittance control");

figure;
subplot(1,2,1);
plot(K_hw,'o');
ylim([0 10]);
title("K hand written");
subplot(1,2,2);
plot(K_schunk,'o');
title("K shcunk");

figure;
subplot(1,2,1)
K_hw_valid = K_hw(~isnan(K_hw) & ~isinf(K_hw));
histogram(K_hw_valid/10, 0:0.001:.2, "Normalization","probability");
[pmf_hw, edges_hw] = histcounts(K_hw_valid/10, "Normalization","probability");
pmf_hw = pmf_hw(pmf_hw > 0);
entropy_hw = -sum(pmf_hw.*log2(pmf_hw));

subplot(1,2,2)
K_schunk_valid = K_schunk(~isnan(K_schunk) & ~isinf(K_schunk));
histogram(K_schunk_valid, 0:0.001:.2, "Normalization","probability");
[pmf_schunk, edges_schunk] = histcounts(K_schunk_valid, "Normalization","probability");
pmf_schunk = pmf_schunk(pmf_schunk > 0);
entropy_schunk = -sum(pmf_schunk.*log2(pmf_schunk));