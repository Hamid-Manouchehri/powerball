clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

save_hand_written_maze_file = "test.mat";  % TODO
% save_hand_written_maze_file = "tahsin_exp5_hw_mat.mat";  % TODO
read_hand_written_maze_mat = "data_tahsin/maze_INK_tahsin_exp_5.mat";  % TODO

hand_written_maze_file_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/IDETC26/mat/";
hand_written_maze_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/IDETC26/";
read_hand_written_maze_mat = hand_written_maze_dir + read_hand_written_maze_mat;

load_hand_written_maze = load(read_hand_written_maze_mat);

x = load_hand_written_maze.pts(:, 1);
y = load_hand_written_maze.pts(:, 2);
t = load_hand_written_maze.pts(:, 3);
t = t(:) - t(1);  % Set time origin to zero
numSamples = size(t, 1);

xDot = zeros(numSamples, 1);
yDot = zeros(numSamples, 1);

for i = 2:numSamples-1
    dt = t(i+1) - t(i-1);
    xDot(i) = (x(i+1) - x(i-1)) / dt;
    yDot(i) = (y(i+1) - y(i-1)) / dt;
end

xDDot = zeros(numSamples, 1);
yDDot = zeros(numSamples, 1);

for i = 2:numSamples-1
    dt = t(i+1) - t(i-1);
    xDDot(i) = (xDot(i+1) - xDot(i-1)) / dt;
    yDDot(i) = (yDot(i+1) - yDot(i-1)) / dt;
end

R     = zeros(numSamples, 1);  % radius of curvature
vNorm = zeros(numSamples, 1);
alpha = 0.;  % TODO

% Calculating radius of curvature:
eps_v = 1e-6;
eps_c = 1e-12;   % to avoid division by zero in curvature term

for i = 2:numSamples-1
    vNorm(i) = norm([xDot(i) yDot(i)]);
    denom = abs(xDot(i)*yDDot(i) - yDot(i)*xDDot(i));

    if vNorm(i) > eps_v && denom > eps_c
        R(i) = vNorm(i)^3 / denom;
        R(i) = R(i) / (1 + alpha*R(i));
    else
        R(i) = NaN;
    end
end

windowSize = 10;   % TODO
overlap = 0;       % TODO
step = windowSize - overlap;

num_windows = floor((numSamples - windowSize) / step) + 1;

beta_hat = NaN(num_windows, 1);
K_hat = NaN(num_windows, 1);
window_center_idx = NaN(num_windows, 1);

window_start_idx = NaN(num_windows, 1);
window_end_idx   = NaN(num_windows, 1);

j = 0;
for i = 1:step:numSamples-windowSize+1

    j = j + 1;
    idx1 = i;
    idx2 = i + windowSize - 1;

    window_start_idx(j) = idx1;
    window_end_idx(j) = idx2;

    vNorm_window = vNorm(idx1:idx2);
    R_window = R(idx1:idx2);

    valid_window = ~isnan(R_window) & ~isnan(vNorm_window) & (R_window > 0) & (vNorm_window > 0);

    if sum(valid_window) >= 2
        p = polyfit(log(R_window(valid_window)), log(vNorm_window(valid_window)), 1);
        beta_hat(j) = p(1);
        K_hat(j) = exp(p(2));
    end

    window_center_idx(j) = floor((idx1 + idx2) / 2);
end

window_center_time = t(window_center_idx);

x_window = cell(num_windows,1);
y_window = cell(num_windows,1);
t_window = cell(num_windows,1);

for i=1:num_windows
    idx1 = window_start_idx(i);
    idx2 = window_end_idx(i);
    
    x_window{i} = x(idx1:idx2);
    y_window{i} = y(idx1:idx2);
    t_window{i} = t(idx1:idx2);
end

% Spatial calculation of entropy:
x_gross_max = 0.6;   % TODO
x_fine_min  = 0.86;  % TODO
y_threshold = 0.5;

% Compute mean X position of each window
x_window_mean = NaN(num_windows, 1);
y_window_mean = NaN(num_windows, 1);
for i = 1:num_windows
    x_window_mean(i) = mean(x_window{i});
    y_window_mean(i) = mean(y_window{i});
end

% Assign zone label per window based on x location: 1=Gross, 2=Transition, 3=Fine
zone_label = zeros(num_windows, 1);
for i = 1:num_windows
    if x_window_mean(i) < x_gross_max
        zone_label(i) = 1;          % Gross
    elseif x_window_mean(i) > x_fine_min
        zone_label(i) = 3;          % Fine
    elseif x_window_mean(i) < x_fine_min && ...
        x_window_mean(i) > x_gross_max && y_window_mean(i) > y_threshold
        zone_label(i) = 2;          % Transition: gross to fine
    else
        zone_label(i) = 4;          % Transition: fine to gross
    end
end

entropy.zone_num  = NaN(4, 1);
entropy.zone_name = ["gross", "gross2fine", "fine", "fine2gross"];

for z = 1:4  % iterate through zone labels
    idx_z = find(zone_label == z & ~isnan(K_hat));
    K_z = K_hat(idx_z);

    if numel(K_z) < 2
        warning('Zone %s has too few windows for entropy.', entropy.zone_name{z});
        continue
    end

    [pmf, edges] = histcounts(K_z/10, "Normalization","probability");
    pmf = pmf(pmf > 0);  % remove zero bins (log(0) guard)
    entropy.zone_num(z) = -sum(pmf .* log2(pmf));
end





% plotting
% -------------------------------------------------------------------------
figure;
plot(x, y, "o");
xlabel("x (m)");
ylabel("y (m)");
xline([x_gross_max, x_fine_min],'r');
title("hand written maze path.");

% figure;
% beta_error = abs(1/3 - beta_hat);
% histogram(beta_hat,-2:0.02:2,'Normalization','probability')
% grid on;
% ylabel('\beta');
% title('Beta error');

save(hand_written_maze_file_dir + save_hand_written_maze_file, ...
     't', 'x', 'y', 'window_center_time', ...
     'entropy', 'beta_hat', 'K_hat', 'R', ...
     't_window', 'x_window', 'y_window');





