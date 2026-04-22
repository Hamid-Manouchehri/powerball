clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/functions/");

read_csv_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/";
save_mat_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/mat/";

save_mat_file = "test.mat";  % TODO

% read_csv_file = "sub4/direct_5_schunk.csv";  % TODO
% controller_flag = "direct";  % TODO; "direct" (constant damping + direct intention) or "indirect"
% 
% data = read_csv_dir + read_csv_file;
% schunk_table = readtable(data);
% 
% 
% % if controller_flag == "direct"
%     t          = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
%     Q          = schunk_table{:,2:7};    % Schunk angular pos
%     Qdot_a     = schunk_table{:,8:13};   % Schunk angular velocity
%     Qdot       = schunk_table{:,14:19};  % Input velocity command (admittance control)
%     FT         = schunk_table{:,20:25};  % Raw FT
%     F_cmd      = schunk_table{:,26:31};  % FT after filtering
%     v_meas     = schunk_table{:,32:37};  % end-effector velocity
%     v_meas_lpf = schunk_table{:,38:43};
%     vel        = schunk_table{:,44:49};  % admittance vel
%     c_des      = schunk_table{:,50:51};  % admittance law
%     Cd         = schunk_table{:,52:57};  % filtered and clamped C
% 
% elseif controller_flag == "indirect"
%     t      = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
%     Q      = schunk_table{:,2:7};    % Schunk angular pos
%     Qdot_a = schunk_table{:,8:13};   % Schunk angular velocity
%     Qdot   = schunk_table{:,14:19};  % Input velocity command (admittance control)
%     FT     = schunk_table{:,20:25};  % Raw FT
%     F_cmd  = schunk_table{:,26:31};  % FT after filtering
%     v_meas = schunk_table{:,32:37};  % end-effector velocity
%     vel    = schunk_table{:,38:43};  % admittance vel
%     rf     = schunk_table{:,44:46};  % filtered reference var
%     F_exp  = schunk_table{:,47:49};  % expected force
%     beta   = schunk_table{:,50};     % Beta
%     Ci     = schunk_table{:,51:59};  % virtual damping
%     Mi_inv = schunk_table{:,60:68};  % inv virtual mass
% 
% end
% 
% schunk_numOfDataSamples = size(t, 1);
% 
% ee_pos    = zeros(schunk_numOfDataSamples,3);
% ee_orient = zeros(schunk_numOfDataSamples,3);
% ee_vel    = zeros(schunk_numOfDataSamples,6);
% 
% % Extracting cartesian space data from schunk:
% for i = 1:schunk_numOfDataSamples
%     ee_vel(i, :) = Qdot_a(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
%     T = FK_schunk_fun(Q(i, :));
%     T = T';
%     ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
%     ee_pos(i, :) = T(4, :);
% end
% 
% 
% %%% plot end-effector pose + speed hitmap
% 
% figure(Name="ee_pos",NumberTitle="off");
% 
% speed = sqrt(v_meas(:,1).^2 + v_meas(:,2).^2);
% 
% scatter(ee_pos(:,1), ee_pos(:,2), 20, speed, 'filled');
% xlabel("x [m]");
% ylabel("y [m]");
% axis equal;
% grid on;
% cb = colorbar;
% colormap jet;
% cb.Label.String = "speed [m/s]";
% title('Trajectory colored by speed');
% 
% 
% figure(Name="T_V_F", NumberTitle="off");
% 
% 
% subplot(2,1,1);
% plot(t, F_cmd(:,1));
% hold on;
% plot(t, F_cmd(:,2));
% ylabel("F_{cmd} [N]");
% legend(["X", "Y"]);
% subplot(2,1,2);
% plot(t, v_meas(:,1));
% hold on;
% plot(t, v_meas(:,2));
% xlabel("Time [s]")
% ylabel("Velocity [m/s]");
% legend(["X", "Y"]);


%%% box plots

task_completion_time_per_trial = NaN(5,1);
task_completion_time_per_controller = NaN(5,4);   % 5 subjects x 4 controllers
mean_force_per_controller = NaN(25,4);            % (5 subjects * 5 trials) x 4

case_name = ["/const_10_", "/const_100_", "/direct_", "/indirect_"];

for controller_idx = 1:4

    if controller_idx < 4
        controller_flag = "direct";
    else
        controller_flag = "indirect";
    end

    for sub_idx = 1:5

        task_completion_time_per_trial(:) = NaN;

        for trl_idx = 1:5

            read_csv_file = "sub" + num2str(sub_idx) + case_name(controller_idx) + ...
                num2str(trl_idx) + "_schunk.csv";

            read_csv_file = read_csv_dir + read_csv_file;
            schunk_table = readtable(read_csv_file);

            if controller_flag == "direct"
                t          = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
                Q          = schunk_table{:,2:7};
                Qdot_a     = schunk_table{:,8:13};
                Qdot       = schunk_table{:,14:19};
                FT         = schunk_table{:,20:25};
                F_cmd      = schunk_table{:,26:31};
                v_meas     = schunk_table{:,32:37};
                v_meas_lpf = schunk_table{:,38:43};
                vel        = schunk_table{:,44:46};
                c_des      = schunk_table{:,47:48};
                Cd         = schunk_table{:,49:54};

            else
                t      = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
                Q      = schunk_table{:,2:7};
                Qdot_a = schunk_table{:,8:13};
                Qdot   = schunk_table{:,14:19};
                FT     = schunk_table{:,20:25};
                F_cmd  = schunk_table{:,26:31};
                v_meas = schunk_table{:,32:37};
                vel    = schunk_table{:,38:43};
                rf     = schunk_table{:,44:46};
                F_exp  = schunk_table{:,47:49};
                beta   = schunk_table{:,50};
                Ci     = schunk_table{:,51:59};
                Mi_inv = schunk_table{:,60:68};
            end

            schunk_numOfDataSamples = size(t, 1);
            ee_pos    = zeros(schunk_numOfDataSamples,3);
            ee_orient = zeros(schunk_numOfDataSamples,3);
            ee_vel    = zeros(schunk_numOfDataSamples,6);

            for i = 1:schunk_numOfDataSamples
                ee_vel(i,:) = Qdot_a(i,:) * transpose(Jacob_schunk_fun(Q(i,:)));
                T = FK_schunk_fun(Q(i,:));
                T = T';
                ee_orient(i,:) = rotm2eul(T(1:3,:), 'ZYX');
                ee_pos(i,:) = T(4,:);
            end

            speed = sqrt(ee_vel(:,1).^2 + ee_vel(:,2).^2);

            v_th = 0.01;
            idx = speed > v_th;

            row_idx = (sub_idx - 1)*5 + trl_idx;

            if any(idx)
                t_start = t(find(idx, 1, 'first'));
                t_end   = t(find(idx, 1, 'last'));
                task_completion_time_per_trial(trl_idx) = t_end - t_start;

                force_mag = sqrt(FT(:,1).^2 + FT(:,2).^2);
                mean_force_per_controller(row_idx, controller_idx) = mean(force_mag(idx));
            else
                task_completion_time_per_trial(trl_idx) = NaN;
                mean_force_per_controller(row_idx, controller_idx) = NaN;
            end

        end

        task_completion_time_per_controller(sub_idx, controller_idx) = ...
            mean(task_completion_time_per_trial, "omitnan");

    end
end

save(save_mat_dir + save_mat_file, ...
    "task_completion_time_per_controller", ...
    "mean_force_per_controller");