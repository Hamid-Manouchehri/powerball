clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

hand_written_maze_file_save = "hand_written_mat_save.mat";
hand_written_maze_file_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";

% hand_written_maze_mat = "maze_INK_data_slow.mat";  % TODO
hand_written_maze_mat = ["maze_INK_data_slow.mat", "maze_INK_data_mid.mat", "maze_INK_data_fast.mat"];  % TODO
hand_written_maze_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/";
hand_written_maze_mat = hand_written_maze_dir + hand_written_maze_mat;

% for k=1:3

    hand_written_maze = load(hand_written_maze_mat(1));
    
    x = hand_written_maze.pts(:, 1);
    y = hand_written_maze.pts(:, 2);
    t = hand_written_maze.pts(:, 3);
    t = t(:) - t(1);
    numSamples = size(t, 1);
    
    xDot = zeros(numSamples, 1);
    yDot = zeros(numSamples, 2);
    
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
    
    j = 1;
    for i = 1:step:numSamples-windowSize+1
    
        idx1 = i;
        idx2 = i + windowSize - 1;
    
        vNorm_window = vNorm(idx1:idx2);
        R_window = R(idx1:idx2);
    
        valid_window = ~isnan(R_window) & ~isnan(vNorm_window) & (R_window > 0) & (vNorm_window > 0);
    
        if sum(valid_window) >= 2
            p = polyfit(log(R_window(valid_window)), log(vNorm_window(valid_window)), 1);
            beta_hat(j) = p(1);
            K_hat(j) = exp(p(2));
        end
    
        window_center_idx(j) = floor((idx1 + idx2) / 2);
        j = j + 1;
    end
    
    window_center_time = t(window_center_idx);
    
    % figure;
    % plot(x, y);
    % xlabel("x (m)");
    % ylabel("y (m)");
    % title("hand written maze path.");
    
    % figure;
    % subplot(1,3,k);
    beta_error = abs(1/3 - beta_hat);
    % plot(window_center_time, beta_error, "o");
    histogram(beta_hat,-2:0.02:2,'Normalization','probability')
    grid on;
    % xlabel('time [s]');
    ylabel('\beta');
    title('Beta error');

% end

save((hand_written_maze_file_dir+hand_written_maze_file_save), ...
     't', 'x', 'y', ...
     'window_center_time', 'beta_hat', ...
     'K_hat', ...
     'R');







