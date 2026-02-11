clc;        close all;          clear;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];  % TODO

%%% Loading data: time, EndEff Position, EndEff Rotation, EndEff Vel, F/T
dataset   = fcn_load_csv(fName);
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
Qdot_lim  = 72*pi/180*ones(6,1);  % rad/s
Qddot_lim = 150*pi/180*ones(6,1);  % rad/s^2
Xdot = fcn_Jacob_schunk(Qdot_lim)*Qdot_lim;  % Extreme cartesian velocity
Xddot = dJ_dt_fun(Qdot_lim, Qddot_lim)*Qdot_lim + fcn_Jacob_schunk(Qdot_lim)*Qddot_lim;  % Extreme cartesian acceleration

for i=1:numDataSamples
    admit_accel(i,1:3) = M_admit\(FT(i,1:3)'-C_admit*admit_vel(i,1:3)');
end

% Tracking errors:
vel_error   = admit_vel(:,1:3) - ee_vel(:,1:3);
accel_error = admit_accel(:,1:3) - ee_accel(:,1:3);

% Normalization:

for i=1:numDataSamples
    XhatDot(i,:)  = vel_error(i,:) ./ Xdot(1:3)';
    XhatDDot(i,:) = accel_error(i,:) ./ Xddot(1:3)';
end

R = M_admit\C_admit;

for i=1:numDataSamples
    difference(i) = norm(XhatDDot(i,:) + R.*XhatDot(i,:));
end

figure
plot(difference);

figure
plot(ee_pos(:,2), ee_pos(:,1),"b."); 
hold on;

epsilon = difference > 40;
plot(ee_pos(epsilon == 1,2), ee_pos(epsilon == 1, 1), 'ro')