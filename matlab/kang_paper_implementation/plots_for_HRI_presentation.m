%{
Generating plots like fig. 7,8,9 as Kang's paper for any experimental task.
%}

clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/functions/");

read_csv_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/";
read_mat_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/mat/";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
read_csv_file = "sub1/direct_5_schunk.csv";  % TODO
controller_flag = "direct";  % TODO; "direct" (constant damping + direct intention) or "indirect"

data = read_csv_dir + read_csv_file;
schunk_table = readtable(data);


if controller_flag == "direct"
    t          = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
    Q          = schunk_table{:,2:7};    % Schunk angular pos
    Qdot_a     = schunk_table{:,8:13};   % Schunk angular velocity
    Qdot       = schunk_table{:,14:19};  % Input velocity command (admittance control)
    FT         = schunk_table{:,20:25};  % Raw FT
    F_cmd      = schunk_table{:,26:31};  % FT after filtering
    v_meas     = schunk_table{:,32:37};  % end-effector velocity
    v_meas_lpf = schunk_table{:,38:43};
    vel        = schunk_table{:,44:49};  % admittance vel
    c_des      = schunk_table{:,50:51};  % admittance law
    Cd         = schunk_table{:,52:57};  % filtered and clamped C

elseif controller_flag == "indirect"
    t      = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
    Q      = schunk_table{:,2:7};    % Schunk angular pos
    Qdot_a = schunk_table{:,8:13};   % Schunk angular velocity
    Qdot   = schunk_table{:,14:19};  % Input velocity command (admittance control)
    FT     = schunk_table{:,20:25};  % Raw FT
    F_cmd  = schunk_table{:,26:31};  % FT after filtering
    v_meas = schunk_table{:,32:37};  % end-effector velocity
    vel    = schunk_table{:,38:43};  % admittance vel
    rf     = schunk_table{:,44:46};  % filtered reference var
    F_exp  = schunk_table{:,47:49};  % expected force
    beta   = schunk_table{:,50};     % Beta
    Ci     = schunk_table{:,51:59};  % virtual damping
    Mi_inv = schunk_table{:,60:68};  % inv virtual mass

end

schunk_numOfDataSamples = size(t, 1);

ee_pos    = zeros(schunk_numOfDataSamples,3);
ee_orient = zeros(schunk_numOfDataSamples,3);
ee_vel    = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qdot_a(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

%%% plot end-effector pose + speed hitmap

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
% title(read_csv_file,'Interpreter', 'none');

figure('Name',"ee_pos",'NumberTitle',"off");

speed = sqrt(v_meas(:,1).^2 + v_meas(:,2).^2);
scatter(ee_pos(:,2), -ee_pos(:,1), 20, speed, 'filled');
xlabel("y [m]");
ylabel("x [m]");
axis equal;
grid on;
cb = colorbar;
colormap jet;
cb.Label.String = "speed [m/s]";
% title(read_csv_file,'Interpreter','none');
fontsize(22, "points");



figure(Name="T_V_F", NumberTitle="off");


subplot(2,1,1);
plot(t, F_cmd(:,1));
hold on;
plot(t, F_cmd(:,2));
ylabel("F_{cmd} [N]");
legend(["X", "Y"]);

% subplot(2,1,2);
% plot(t, v_meas(:,1));
% hold on;
% plot(t, v_meas(:,2));
% xlabel("Time [s]")
% ylabel("Velocity [m/s]");
% legend(["X", "Y"]);


subplot(2,1,2);

yyaxis left
plot(t, v_meas(:,1), 'Color', "b");
hold on;
plot(t, v_meas(:,2), 'Color',"g");
ylabel("Velocity [m/s]");

yyaxis right
plot(t, sqrt(c_des(:,1).^2 + c_des(:,2).^2), 'r');
ylabel("c_{des}");

xlabel("Time [s]");
legend(["X vel", "Y vel", "c_{des}"]);





figure('Name',"ee_pos_direct_damping",'NumberTitle',"off");

damping_mag = sqrt(c_des(:,1).^2 + c_des(:,2).^2);
scatter(ee_pos(:,2), -ee_pos(:,1), 20, damping_mag, 'filled');
xlabel("y [m]");
ylabel("x [m]");
axis equal;
grid on;
cb = colorbar;
colormap jet;
cb.Label.String = "damping mag [Ns/m]";
% title(read_csv_file,'Interpreter','none');
fontsize(22,"points");







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

read_mat_file = "test.mat";  % TODO
matData = load(read_mat_dir + read_mat_file);

figure('Name','boxplots','NumberTitle','off');

subplot(1,2,1);
boxplot(matData.task_completion_time_per_controller, ...
    'Labels', {'min (c=10)','max (c=100)','direct','indirect'});
ylabel('Task Time [s]');
% title('Task completion time per controller');

subplot(1,2,2);
boxplot(matData.mean_force_per_controller, ...
    'Labels', {'min (c=10)','max (c=100)','direct','indirect'});
ylabel('Mean Force [s]');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%