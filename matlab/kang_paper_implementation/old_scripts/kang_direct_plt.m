clc; clear; close all;

read_mat_file = ["1d_var_damping_rate_limit_schunk.mat",...
                 "1d_const_20_schunk.mat", ...
                 "1d_const_100_schunk.mat"];
matDir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/admittance/mat/";



figure(Name="V_F_D", NumberTitle="off");



subplot(2,1,1);
load(matDir+read_mat_file(1));
vel_max = max(-vel(:,1));
vel_max_idx = find(-vel(:,1) == vel_max);
plot(-vel(1:vel_max_idx,1), Cd(1:vel_max_idx,1), ".");
subplot(2,1,2);
F_min_x = Cd(:,1).*(-vel(:,1));
plot(-vel(1:vel_max_idx,1), F_min_x(1:vel_max_idx), ".");


subplot(2,1,1);
load(matDir+read_mat_file(2));
hold on;
plot(-vel(:,1), Cd(:,1));
subplot(2,1,2);
hold on;
F_min_x = Cd(:,1).*(-vel(:,1));
plot(-vel(:,1), F_min_x);


subplot(2,1,1);
load(matDir+read_mat_file(3));
hold on;
plot(-vel(:,1), Cd(:,1));
xlabel("Vel X");
ylabel("Damping X");
% xlim([0 0.18]);
ylim([0 110]);
legend(["var C", "min C", "max C"]);
subplot(2,1,2);
hold on;
F_min_x = Cd(:,1).*(-vel(:,1));
plot(-vel(:,1), F_min_x);
xlabel("Vel X");
ylabel("F_{min} X");
legend(["var C", "min C", "max C"]);
yline(3, '--', 'F_{std}');




figure(Name="T_F_D", NumberTitle="off");



subplot(2,1,1);
load(matDir+read_mat_file(1));
plot(t, Cd(:,1), "r.");
subplot(2,1,2);
yyaxis left
plot(t, -F_cmd(:,1), "r.");

subplot(2,1,1);
load(matDir+read_mat_file(2));
hold on;
plot(t, Cd(:,1));
subplot(2,1,2);
yyaxis left
hold on;
plot(t, -F_cmd(:,1));

subplot(2,1,1);
load(matDir+read_mat_file(3));
hold on;
plot(t, Cd(:,1));
xlabel("Time [s]");
ylabel("Damping X");
ylim([0 110]);
legend(["var C", "min C", "max C"]);
subplot(2,1,2);
yyaxis left
plot(t, -F_cmd(:,1), 'Color', "g");
xlabel("Time [s]");
ylabel("F_{cmd} X");
yyaxis right
hold on;
plot(t,-vel(:,1));
ylabel("admittance vel (m/s)");
legend(["var C", "min C", "max C", "vel"]);


figure(Name="Cd", NumberTitle="off");


load(matDir+read_mat_file(1));
subplot(2,1,1);
plot(t, c_des);
subplot(2,1,2);
plot(t, Cd);



% For HRI presentation
figure;
% subplot(2,1,1);
load(matDir+read_mat_file(1));
vel_max = max(-vel(:,1));
vel_max_idx = find(-vel(:,1) == vel_max);
plot(-vel(1:vel_max_idx,1), Cd(1:vel_max_idx,1), ".");
% subplot(2,1,1);
load(matDir+read_mat_file(2));
hold on;
plot(-vel(:,1), Cd(:,1));
% subplot(2,1,1);
load(matDir+read_mat_file(3));
hold on;
plot(-vel(:,1), Cd(:,1));
xlabel("Vel X");
ylabel("Damping X");
% xlim([0 0.18]);
ylim([0 110]);
legend(["var C", "min C", "max C"]);
fontsize(26,"points");



figure;
load(matDir+read_mat_file(1));
plot(t, Cd(:,1), "r.");
% subplot(2,1,1);
load(matDir+read_mat_file(2));
hold on;
plot(t, Cd(:,1));
load(matDir+read_mat_file(3));
hold on;
plot(t, Cd(:,1));
xlabel("Time [s]");
ylabel("Damping X");
ylim([0 110]);
legend(["var C", "min C", "max C"]);
fontsize(26,"points")