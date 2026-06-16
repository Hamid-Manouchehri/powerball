%{
Read post-processed ".mat" file data for table 2 in Kang paper
%}
clc; clear; close all

read_mat_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/mat/";

BIG_CIRCLE_CIRCUMFERENCE = 0.3142;  % meter
SQUARE_CIRCUMFERENCE = 0.32;  % meter


circle_direct_task_time   = NaN(5,5);
square_direct_task_time   = NaN(5,5);
circle_indirect_task_time = NaN(5,5);
square_indirect_task_time = NaN(5,5);

circle_direct_mean_force   = NaN(5,5);
square_direct_mean_force   = NaN(5,5);
circle_indirect_mean_force = NaN(5,5);
square_indirect_mean_force = NaN(5,5);

circle_direct_path_length   = NaN(5,5);
square_direct_path_length   = NaN(5,5);
circle_indirect_path_length = NaN(5,5);
square_indirect_path_length = NaN(5,5);

for sub_idx = 1:5

    read_mat_file_path_length = "sub" + num2str(sub_idx) + "_path_time_mean_force_length.mat";
    data = load(read_mat_dir + read_mat_file_path_length);

    circle_direct_task_time(:,sub_idx)   = data.task_time_matrix(11:15, 1);
    square_direct_task_time(:,sub_idx)   = data.task_time_matrix(11:15, 2);
    circle_indirect_task_time(:,sub_idx) = data.task_time_matrix(16:20, 1);
    square_indirect_task_time(:,sub_idx) = data.task_time_matrix(16:20, 2);

    circle_direct_mean_force(:,sub_idx)   = data.mean_force_matrix(11:15, 1);
    square_direct_mean_force(:,sub_idx)   = data.mean_force_matrix(11:15, 2);
    circle_indirect_mean_force(:,sub_idx) = data.mean_force_matrix(16:20, 1);
    square_indirect_mean_force(:,sub_idx) = data.mean_force_matrix(16:20, 2);

    circle_direct_path_length(:,sub_idx)   = data.path_length_matrix(11:15, 1);
    square_direct_path_length(:,sub_idx)   = data.path_length_matrix(11:15, 2);
    circle_indirect_path_length(:,sub_idx) = data.path_length_matrix(16:20, 1);
    square_indirect_path_length(:,sub_idx) = data.path_length_matrix(16:20, 2);

end

report.circle_direct.task_time   = mean(circle_direct_task_time,   'all', 'omitnan');
report.square_direct.task_time   = mean(square_direct_task_time,   'all', 'omitnan');
report.circle_indirect.task_time = mean(circle_indirect_task_time, 'all', 'omitnan');
report.square_indirect.task_time = mean(square_indirect_task_time, 'all', 'omitnan');

report.circle_direct.mean_force   = mean(circle_direct_mean_force,   'all', 'omitnan');
report.square_direct.mean_force   = mean(square_direct_mean_force,   'all', 'omitnan');
report.circle_indirect.mean_force = mean(circle_indirect_mean_force, 'all', 'omitnan');
report.square_indirect.mean_force = mean(square_indirect_mean_force, 'all', 'omitnan');

report.circle_direct.path_length   = mean(circle_direct_path_length,   'all', 'omitnan');
report.square_direct.path_length   = mean(square_direct_path_length,   'all', 'omitnan');
report.circle_indirect.path_length = mean(circle_indirect_path_length, 'all', 'omitnan');
report.square_indirect.path_length = mean(square_indirect_path_length, 'all', 'omitnan');

%%  HRI Presentation:
circle_damp_10_task_time   = NaN(5,5);
circle_damp_100_task_time  = NaN(5,5);
circle_direct_task_time    = NaN(5,5);
circle_indirect_task_time  = NaN(5,5);

circle_damp_10_mean_force   = NaN(5,5);
circle_damp_100_mean_force  = NaN(5,5);
circle_direct_mean_force    = NaN(5,5);
circle_indirect_mean_force  = NaN(5,5);

circle_damp_10_path_length   = NaN(5,5);
circle_damp_100_path_length  = NaN(5,5);
circle_direct_path_length    = NaN(5,5);
circle_indirect_path_length  = NaN(5,5);

for sub_idx = 1:5

    read_mat_file_path_length = "sub" + num2str(sub_idx) + "_path_time_mean_force_length.mat";
    data = load(read_mat_dir + read_mat_file_path_length);

    % circle only
    circle_damp_10_task_time(:,sub_idx)   = data.task_time_matrix(1:5, 1);
    circle_damp_100_task_time(:,sub_idx)  = data.task_time_matrix(6:10, 1);
    circle_direct_task_time(:,sub_idx)    = data.task_time_matrix(11:15, 1);
    circle_indirect_task_time(:,sub_idx)  = data.task_time_matrix(16:20, 1);

    circle_damp_10_mean_force(:,sub_idx)   = data.mean_force_matrix(1:5, 1);
    circle_damp_100_mean_force(:,sub_idx)  = data.mean_force_matrix(6:10, 1);
    circle_direct_mean_force(:,sub_idx)    = data.mean_force_matrix(11:15, 1);
    circle_indirect_mean_force(:,sub_idx)  = data.mean_force_matrix(16:20, 1);

    circle_damp_10_path_length(:,sub_idx)   = data.path_length_matrix(1:5, 1);
    circle_damp_100_path_length(:,sub_idx)  = data.path_length_matrix(6:10, 1);
    circle_direct_path_length(:,sub_idx)    = data.path_length_matrix(11:15, 1);
    circle_indirect_path_length(:,sub_idx)  = data.path_length_matrix(16:20, 1);

end

% flatten to 25x1 for each controller
tt_min   = circle_damp_10_task_time(:);
tt_max   = circle_damp_100_task_time(:);
tt_dir   = circle_direct_task_time(:);
tt_ind   = circle_indirect_task_time(:);

mf_min   = circle_damp_10_mean_force(:);
mf_max   = circle_damp_100_mean_force(:);
mf_dir   = circle_direct_mean_force(:);
mf_ind   = circle_indirect_mean_force(:);

pl_min   = circle_damp_10_path_length(:);
pl_max   = circle_damp_100_path_length(:);
pl_dir   = circle_direct_path_length(:);
pl_ind   = circle_indirect_path_length(:);

figure('Name','Circular path boxplots','NumberTitle','off');

subplot(1,3,1);
boxplot([tt_min, tt_max, tt_dir, tt_ind], ...
    'Labels', {'Min','Max','Direct','Indirect'});
ylabel('Task Time [s]');
% title('Circular motion');
grid on;

subplot(1,3,2);
boxplot([mf_min, mf_max, mf_dir, mf_ind], ...
    'Labels', {'Min','Max','Direct','Indirect'});
ylabel('Mean Force [N]');
grid on;

subplot(1,3,3);
boxplot([pl_min, pl_max, pl_dir, pl_ind], ...
    'Labels', {'Min','Max','Direct','Indirect'});
ylabel('Path Length [m]');
grid on;
fontsize(22,"points");