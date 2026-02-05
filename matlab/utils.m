clc; clear;


function T = FK_schunk(Q)
    %   Q  : 6x1 (or 1x6) joint vector [q1..q6]
    %   a1,a2,a3,d6 : scalar link parameters used in the original C++ code
    %   T  : 3x4 matrix
    
    a1 = .205;
    a2 = .350;
    a3 = .305;
    d6 = .085+.155; % flang surface + gripper height

    q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4); q5 = Q(5); q6 = Q(6);
    
    T = [ ...
    cos(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)) ...
     - sin(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)), ...
    - cos(q6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*sin(q4) + cos(q1)*sin(q2)*sin(q3)*sin(q4)) ...
     - sin(q6)*(cos(q1)*cos(q3)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q5) - cos(q5)*sin(q1)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)), ...
    cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), ...
    a3*cos(q1)*cos(q2)*sin(q3) - a2*cos(q1)*sin(q2) - a3*cos(q1)*cos(q3)*sin(q2) ...
     - d6*sin(q1)*sin(q4)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q5)*sin(q3) - d6*cos(q1)*cos(q3)*cos(q5)*sin(q2) ...
     + d6*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + d6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5); ...
    
    cos(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)) ...
     - sin(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)), ...
    - cos(q6)*(cos(q2)*cos(q3)*sin(q1)*sin(q4) - cos(q1)*cos(q4) + sin(q1)*sin(q2)*sin(q3)*sin(q4)) ...
     - sin(q6)*(cos(q1)*cos(q5)*sin(q4) - cos(q2)*sin(q1)*sin(q3)*sin(q5) + cos(q3)*sin(q1)*sin(q2)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)), ...
    cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), ...
    a3*cos(q2)*sin(q1)*sin(q3) - a2*sin(q1)*sin(q2) - a3*cos(q3)*sin(q1)*sin(q2) ...
     + d6*cos(q1)*sin(q4)*sin(q5) + d6*cos(q2)*cos(q5)*sin(q1)*sin(q3) - d6*cos(q3)*cos(q5)*sin(q1)*sin(q2) ...
     + d6*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + d6*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5); ...
    
    - cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) ...
     - sin(q2 - q3)*sin(q4)*sin(q6), ...
      sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) ...
     - sin(q2 - q3)*cos(q6)*sin(q4), ...
    cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5), ...
    a1 + a2*cos(q2) + a3*cos(q2)*cos(q3) + a3*sin(q2)*sin(q3) ...
     + d6*cos(q2)*cos(q3)*cos(q5) + d6*cos(q5)*sin(q2)*sin(q3) - d6*cos(q2)*cos(q4)*sin(q3)*sin(q5) + d6*cos(q3)*cos(q4)*sin(q2)*sin(q5) ...
    ];
end



function J = Jacob_schunk(Q)
    %   Q: 6x1 (or 1x6) joint vector [q1..q6]
    %   J: 6x6 Jacobian
    
    q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4); q5 = Q(5); q6 = Q(6); %#ok<NASGU>
    
    J = [ ...
     (sin(q1)*(61*sin(q2-q3) + 70*sin(q2)))/200, -(cos(q1)*(61*cos(q2-q3) + 70*cos(q2)))/200, (61*cos(q2-q3)*cos(q1))/200, 0, 0, 0; ...
    -(cos(q1)*(61*sin(q2-q3) + 70*sin(q2)))/200, -(sin(q1)*(61*cos(q2-q3) + 70*cos(q2)))/200, (61*cos(q2-q3)*sin(q1))/200, 0, 0, 0; ...
     0, -(61*sin(q2-q3))/200 - (7*sin(q2))/20, (61*sin(q2-q3))/200, 0, 0, 0; ...
     0,  sin(q1), -sin(q1), -sin(q2-q3)*cos(q1), ...
        -cos(q4)*sin(q1) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q1)*sin(q2)*sin(q3)*sin(q4), ...
         cos(q1)*cos(q2)*cos(q5)*sin(q3) - sin(q1)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5); ...
     0, -cos(q1),  cos(q1), -sin(q2-q3)*sin(q1), ...
         cos(q1)*cos(q4) - cos(q2)*cos(q3)*sin(q1)*sin(q4) - sin(q1)*sin(q2)*sin(q3)*sin(q4), ...
         cos(q1)*sin(q4)*sin(q5) + cos(q2)*cos(q5)*sin(q1)*sin(q3) - cos(q3)*cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) + cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5); ...
     1, 0, 0, cos(q2-q3), -sin(q2-q3)*sin(q4), ...
         cos(q2)*cos(q3)*cos(q5) + cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5) ...
    ];
end

function [t, ee_pos, ee_rot, ee_vel, FT_xy_amp] = extract_csv(csvFile)

    opts = detectImportOptions(csvFile, 'Delimiter', ',');
    opts.VariableNamingRule = "preserve";   % keep original header text
    tsCol = opts.VariableNames{contains(opts.VariableNames, "TimeStamp")};
    opts = setvartype(opts, tsCol, "string");
    opts = setvaropts(opts, tsCol, 'WhitespaceRule', 'trim');
    
    data = readtable(csvFile, opts);
    t = datetime(data.TimeStamp, 'InputFormat', 'HH:mm:ss:SSSSSS');
    t = seconds(t - t(1));
    Q = data{:, {'Q1','Q2','Q3','Q4','Q5','Q6'}};
    dQ = data{:, {'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6'}};
    FT = data{:, {'FT1','FT2','FT3','FT4','FT5','FT6'}};
    
    FT_xy_amp = sqrt(FT(:,1).^2 + FT(:,2).^2);
    
    numRowQ = size(Q, 1);
    ee_rot = zeros(3, numRowQ);
    ee_pos = zeros(3, numRowQ);
    ee_vel = zeros(6, numRowQ);
    
    for i = 1:numRowQ
        ee_vel(:, i) = Jacob_schunk(Q(i, :))*dQ(i, :).';
        T = FK_schunk(Q(i, :));
        ee_rot(:, i) = rotm2eul(T(:, 1:3), 'ZYX');  % [yaw pitch roll] in radians
        ee_pos(:, i) = T(:, 4);
    end
end



csvFile_damp_20 = "../data/admittance_control/test_1_020326/maze_damp_20.000000_01.csv";  % TODO
csvFile_damp_50 = "../data/admittance_control/test_1_020326/maze_damp_50.000000_01.csv";  % TODO
csvFile_damp_80 = "../data/admittance_control/test_1_020326/maze_damp_80.000000_01.csv";  % TODO


[t_damp20, ee_pos_damp20, ee_rot_damp20, ee_vel_damp20, FT_xy_amp_damp20] = ...
     extract_csv(csvFile_damp_20);
[t_damp50, ee_pos_damp50, ee_rot_damp50, ee_vel_damp50, FT_xy_amp_damp50] = ...
     extract_csv(csvFile_damp_50);
[t_damp80, ee_pos_damp80, ee_rot_damp80, ee_vel_damp80, FT_xy_amp_damp80] = ...
     extract_csv(csvFile_damp_80);


%%%% plots:
%%% Pos
%figure(Name="pos", NumberTitle="off");
figure
subplot(3, 1, 1); hold on;
plot(t_damp20', ee_pos_damp20(1,:),'.'); 
%title("damp=20")
xlabel('time (s)');
ylabel('pos (m)');
grid on;


%{
subplot(3, 1, 2); hold on;
plot(t_damp50, ee_pos_damp50.'); 
title("damp=50")
xlabel('time (s)');
ylabel('pos (m)');
legend("x", "y", "z");
grid on;

subplot(3, 1, 3); hold on;
plot(t_damp80, ee_pos_damp80.'); 
title("damp=80")
xlabel('time (s)');
ylabel('pos (m)');
legend("x", "y", "z");
grid on;


%%% Vel
figure(Name="vel", NumberTitle="off");
subplot(3, 1, 1); hold on;
plot(t_damp20, ee_vel_damp20.'); 
title("damp=20")
xlabel('time (s)');
ylabel('vel (m/s)');
legend("dx", "dy", "dz");
grid on;

subplot(3, 1, 2); hold on;
plot(t_damp50, ee_vel_damp50.'); 
title("damp=50")
xlabel('time (s)');
ylabel('vel (m/s)');
legend("dx", "dy", "dz");
grid on;

subplot(3, 1, 3); hold on;
plot(t_damp80, ee_vel_damp80.'); 
title("damp=80")
xlabel('time (s)');
ylabel('vel (m/s)');
legend("dx", "dy", "dz");
grid on;


%%% FT
figure(Name="F/T", NumberTitle="off");
subplot(3, 1, 1)
plot(t_damp20, FT_xy_amp_damp20); 
title('damp=20'); 
xlabel('time (s)');
ylabel('Force (N)');
legend("Fxy");

subplot(3, 1, 2)
plot(t_damp50, FT_xy_amp_damp50); 
title('damp=50'); 
xlabel('time (s)');
ylabel('Force (N)');
legend("Fxy");

subplot(3, 1, 3)
plot(t_damp80, FT_xy_amp_damp80); 
title('damp=80'); 
xlabel('time (s)');
ylabel('Force (N)');
legend("Fxy");


figure;
plot(ee_pos_damp20(1,:), ee_pos_damp20(2,:))
%}