clc;        clear;         %close all;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp10 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];  % TODO
fName_damp100 = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO

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
FTdot          = zeros(numDataSamples,3);
theta_F        = zeros(numDataSamples,1);


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
    FTdot(i,:) = (FT(i+1,1:3) - FT(i-1,1:3)) ./ (2*dt(i));  % central diff
    theta_F(i) = tanh(FT(i,2)/FT(i,1));
end

figure;
plot(theta_F);
ylabel("theta (rad)");

figure;
subplot(1,2,1);
plot(FT(:, 1:3));
legend("Fx", "Fy")
title("force");

subplot(1,2,2);
plot(FTdot);
legend("Fdx", "Fdy")
title("diff force")