%{
Read schunk data + time anotated data files and extract path time (s), mean
force (N), and length (m) for the (big) circular and square shapes.
%}
clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/functions/");

read_csv_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/";

% CHANGE BOTH:
% save_mat_file = "mat/sub5_path_time_mean_force_length.mat";  % TODO
% subject_name = "sub5";  % TODO: ["sub1", "sub2", "sub3", "sub4", "sub5"]

controller_file_name = ["const_10_", "const_100_", "direct_", "indirect_"];
time_anot_file_name = ["_damp_10.csv", "_damp_100.csv", "_direct.csv", "_indirect.csv"];

task_time_matrix = NaN(20,2);    % rows=(4 controllers*5 trials), cols=[circle square]
mean_force_matrix = NaN(20,2);   % rows=(4 controllers*5 trials), cols=[circle square]
path_length_matrix = NaN(20,2);  % (4 controllers * 5 trials) * 2 (big_cicle, square)

for controller_idx = 1:4

    path_time_data_file = subject_name + "/" +  subject_name + ...
        time_anot_file_name(controller_idx);
    read_csv_file = read_csv_dir + path_time_data_file;
    path_time_data = readtable(read_csv_file);

    for trial_idx = 1:5

        circle_time = path_time_data{trial_idx,1:2};
        square_time = path_time_data{trial_idx,5:6};

        schunk_data_file = subject_name + "/" + controller_file_name(controller_idx) + ...
            num2str(trial_idx) + "_schunk.csv";
        read_csv_file = read_csv_dir + schunk_data_file;
        schunk_data = readtable(read_csv_file);

        t      = (schunk_data.Time_us - schunk_data.Time_us(1)) / 1e6;
        Q      = schunk_data{:,2:7};
        Qdot_a = schunk_data{:,8:13};
        FT     = schunk_data{:,20:25};   % use F_cmd instead if that is what you want

        N = size(t,1);
        ee_pos = zeros(N,3);
        ee_vel = zeros(N,6);

        for i = 1:N
            ee_vel(i,:) = Qdot_a(i,:) * transpose(Jacob_schunk_fun(Q(i,:)));
            T = FK_schunk_fun(Q(i,:));
            T = T';
            ee_pos(i,:) = T(4,:);
        end

        idx_circle = find(t >= circle_time(1) & t <= circle_time(2));
        idx_square = find(t >= square_time(1) & t <= square_time(2));

        % circle path length
        dx_c = diff(ee_pos(idx_circle,1));
        dy_c = diff(ee_pos(idx_circle,2));
        big_circle_length = sum(sqrt(dx_c.^2 + dy_c.^2));

        % square path length
        dx_s = diff(ee_pos(idx_square,1));
        dy_s = diff(ee_pos(idx_square,2));
        square_length = sum(sqrt(dx_s.^2 + dy_s.^2));

        % task time
        circle_task_time = t(idx_circle(end)) - t(idx_circle(1));
        square_task_time = t(idx_square(end)) - t(idx_square(1));

        % mean force magnitude
        force_mag = sqrt(FT(:,1).^2 + FT(:,2).^2);
        circle_mean_force = mean(force_mag(idx_circle));
        square_mean_force = mean(force_mag(idx_square));

        row_idx = (controller_idx-1)*5 + trial_idx;

        path_length_matrix(row_idx,:) = [big_circle_length, square_length];
        task_time_matrix(row_idx,:)   = [circle_task_time, square_task_time];
        mean_force_matrix(row_idx,:)  = [circle_mean_force, square_mean_force];

    end
end

save(read_csv_dir + save_mat_file, "task_time_matrix", ...
                                   "mean_force_matrix", ...
                                   "path_length_matrix");