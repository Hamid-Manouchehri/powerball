clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

load('./data/admittance/IDETC26/mat/IDETC_2026_data_modified_2.mat')  % TODO

format short

% sub_idx = 4;
% damping_idx = 2;
trial_idx = 5;  % TODO

plt_idx = 0;
f = NaN(13, 16);

for sub_idx = 1:4
    for damping_idx = 1:4

        plt_idx = plt_idx + 1;

        sample_stroke = data(sub_idx, damping_idx).posData{trial_idx};
        num_points = numel(sample_stroke);
        num_points = num_points / 4;

        t = sample_stroke(1:num_points);
        t(end) = [];
        x = sample_stroke(num_points:2*num_points);
        x(1) = []; x(end) = [];
        y = sample_stroke(2*num_points:3*num_points);
        y(1) = []; y(end) = [];

        sample_stroke_win = data(sub_idx, damping_idx).winPosData{trial_idx};
        num_win = numel(sample_stroke_win);
        num_win = num_win / 3;

        winT = sample_stroke_win(1:num_win);
        winT(end) = [];
        winX = sample_stroke_win(num_win:2*num_win);
        winX(1) = []; winX(end) = [];
        winY = sample_stroke_win(2*num_win:end);
        winY(1) = []; winY(end) = [];

        % figure;
        % plot(x,y,'o')

        winSize = numel(winT{1});
        f10 = zeros(numel(winT), 1);
        f11 = zeros(numel(winT), 1);
        theta_p = zeros(numel(winT),1);

        % for i=1:numel(winT)

        % delX = diff(winX{i});
        % delY = diff(winY{i});
        %
        % for ii=2:winSize-1
        %     theta_p(ii-1) = atan((delX(ii)*delY(ii-1) - delX(ii-1)*delY(ii)) / ...
        %         (delX(ii)*delX(ii-1) + delY(ii)*delY(ii-1)));
        % end

        % theta_p(isnan(theta_p))=0;


        % f10(i) = 1/num_points * sum(abs(theta_p));
        % f11(i) = 1/num_points * sum(theta_p.^2);
        % f(:,i) = rubine_fun(winX{i}, winY{i}, winT{i});

        % end

        % winX_mean = zeros(num_win-1, 1);
        % winY_mean = zeros(num_win-1, 1);
        %
        % for iii=1:num_win-1
        %     winX_mean(iii) = mean(winX{iii});
        %     winY_mean(iii) = mean(winY{iii});
        % end

        f(:, plt_idx) = rubine_fun(x, y, t);
        subplot(4,4,plt_idx);
        bar(f(:, plt_idx));
        ylim([-100 900])


    end
end
