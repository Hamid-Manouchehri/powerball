clc; clear; close all;
csv_file       = './data/schunk_data/S1_low_clover_schunk.csv';  % TODO
frame_interval = 10;    % TODO: highlight every Nth point
pause_time     = 0.001;  % TODO: animation speed (seconds)
marker_size    = 8;     % TODO: current position marker size
highlight_size = 15;    % TODO: interval highlight marker size

[t, ee_pos, ~, ~] = schunk_ee_pos_fun(csv_file);
x = ee_pos(:,2);
y = -ee_pos(:,1);

figure(1);
xlim([min(x) max(x)]);
ylim([min(y) max(y)]);
h_trail   = plot(x(1), y(1), 'b-', 'LineWidth', 2); hold on;
h_current = plot(x(1), y(1), 'bo', 'MarkerSize', marker_size, 'MarkerFaceColor', 'b');

for i = 1:length(x)
    set(h_trail,   'XData', x(1:i), 'YData', y(1:i));
    set(h_current, 'XData', x(i),   'YData', y(i));
    % if mod(i, frame_interval) == 0
    %     plot(x(i), y(i), 'g*', 'MarkerSize', highlight_size);
    % end
    title(['Frame ' num2str(i) ' / ' num2str(length(x)) '      time: ' ...
        num2str(t(i))]);
    drawnow limitrate;  % faster than pause
end