clc; clear; close all;

read_schunk_file = "midDamp_50_schunk.mat";  % TODO
read_hand_written_file = "hand_written_mat.mat";  % TODO
dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";
read_schunk_file = dir + read_schunk_file;
read_hand_written_file = dir + read_hand_written_file;

load_hand_written_drawing = load(read_hand_written_file);
load_schunk_admittance = load(read_schunk_file);

% hand-written (hw) data:
t_hw = load_hand_written_drawing.t;
x_hw = load_hand_written_drawing.x;
y_hw = load_hand_written_drawing.y;
R_hw = load_hand_written_drawing.R;
K_hw = load_hand_written_drawing.K_hat;
beta_hw = load_hand_written_drawing.beta_hat;
window_center_time_hw = load_hand_written_drawing.window_center_time;

% Schunk admittance control data:
t_schunk = load_schunk_admittance.schunk_time_s;
x_schunk = load_schunk_admittance.x;
y_schunk = load_schunk_admittance.y;
R_schunk = load_schunk_admittance.R;
K_schunk = load_schunk_admittance.K_hat;
beta_schunk = load_schunk_admittance.beta_hat;
window_center_time_schunk = load_schunk_admittance.window_center_time;

figure;
subplot(1,2,1);
plot(x_hw, y_hw, "r");
xlabel("x (m)"); ylabel("y (m)");
title("hand-written path.")

subplot(1,2,2);
plot(y_schunk, x_schunk, "r")
xlabel("y (m)"); ylabel("x (m)");
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
% K_hw = K_hw(~isnan(K_hw) & ~isinf(K_hw));
histogram(K_hw/10, 0:0.001:.2, "Normalization","probability");

lim_transient_hw_x = [0.8 0.84];
for i=1:3
    [pmf_hw, edges_hw] = histcounts(K_hw/10, "Normalization","probability");
    pmf_hw = pmf_hw(pmf_hw > 0);
    entropy_hw = -sum(pmf_hw.*log2(pmf_hw));
end



subplot(1,2,2)
% K_schunk = K_schunk(~isnan(K_schunk) & ~isinf(K_schunk));
histogram(K_schunk, 0:0.001:.2, "Normalization","probability");

lim_transient_schunk_y = [0.1 0.12];
[pmf_schunk, edges_schunk] = histcounts(K_schunk, "Normalization","probability");
pmf_schunk = pmf_schunk(pmf_schunk > 0);
entropy_schunk = -sum(pmf_schunk.*log2(pmf_schunk));