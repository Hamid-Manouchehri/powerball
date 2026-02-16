clc;        clear;         %close all;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp10 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];  % TODO
fName_damp100 = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO

% fcn_DataAnalysis_improved_heuristic(fName_damp10);
% fcn_DataAnalysis_improved_heuristic(fName_damp100);

dataset   = fcn_load_csv(fName_damp100);
t         = dataset.timesteps; 
Q         = dataset.q; 
dQ        = dataset.qd; 
FT        = dataset.FT;
admit_vel = dataset.vel;  % admittance control velocity; cartesian space

%%% change to cartesian-space
numDataSamples = size(Q, 1);
ee_pos         = zeros(numDataSamples,3);
ee_rot         = zeros(numDataSamples,3);
ee_vel         = zeros(numDataSamples,6);
ee_accel       = zeros(numDataSamples,6);
admit_accel    = zeros(numDataSamples,6);
difference     = zeros(numDataSamples,1);
Xdot           = zeros(numDataSamples,6);
Xddot          = zeros(numDataSamples,6);
XhatDot        = zeros(numDataSamples,3);
XhatDDot       = zeros(numDataSamples,3);

for i = 1:numDataSamples
    ee_vel(i, :) = dQ(i, :)*transpose(fcn_Jacob_schunk(Q(i, :)));
    T = fcn_FK_schunk(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);  % measured end-effector velocity
end

dt = diff(t);
M_admit = 0.3*diag([1, 1, 1]);
C_admit = 10*diag([1.2, 1.0, 1.0]);

for i=2:numDataSamples-1
    ee_accel(i,1:3) = (ee_vel(i+1,1:3) - ee_vel(i-1,1:3)) ./ (2*dt(i));  % calculated end-effector acceleration; Central diff
end

% based on Schunk datasheet:
Q_max = 150*pi/180*ones(6,1);  % rad
Qdot_max  = 72*pi/180*ones(6,1);  % rad/s
Qddot_max = 150*pi/180*ones(6,1);  % rad/s^2

for i=1:numDataSamples
    Xdot(i,:) = fcn_Jacob_schunk(Q(i,:))*Qdot_max;  % Extreme cartesian velocity
    Xddot(i,:) = dJ_dt_fun(Q(i,:)', dQ(i,:)')*Qdot_max + fcn_Jacob_schunk(Q(i,:))*Qddot_max;  % Extreme cartesian acceleration
end

for i=1:numDataSamples  % xdd = M^-1*(f_e - C*xd)
    admit_accel(i,1:3) = M_admit\(FT(i,1:3)'-C_admit*admit_vel(i,1:3)');
end

% Tracking errors:
vel_error   = admit_vel(:,1:3) - ee_vel(:,1:3);
accel_error = admit_accel(:,1:3) - ee_accel(:,1:3);

% Normalization:
for i=1:numDataSamples
    XhatDot(i,:)  = vel_error(i,:) ./ Xdot(i,1:3);
    XhatDDot(i,:) = accel_error(i,:) ./ Xddot(i,1:3);
end

R = M_admit\C_admit;

for i=1:numDataSamples
    difference(i) = norm(XhatDDot(i,:) + R.*XhatDot(i,:));
end

figure
plot(difference);
title("abs of the heuristic error")

figure
plot(ee_pos(:,2), ee_pos(:,1),"b."); 
hold on;

epsilon = difference > 80;  % TODO
plot(ee_pos(epsilon == 1,2), ee_pos(epsilon == 1, 1), 'ro')