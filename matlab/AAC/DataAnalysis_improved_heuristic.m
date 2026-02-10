clc;        close all;          clear

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_2_020426/';
fName = [dataDirName 'et_maze_damp_10.000000_01.csv'];  % TODO

%%% Loading data: time, EndEff Position, EndEff Rotation, EndEff Vel, F/T
dataset = load_csv(fName);
t = dataset.timesteps; 
Q = dataset.q; 
dQ = dataset.qd; 
FT = dataset.FT;

%%% change to cartesian-space
numDataSamples = size(Q, 1);
ee_pos         = zeros(numDataSamples,3);
ee_rot         = zeros(numDataSamples,3);
ee_vel         = zeros(numDataSamples,6);
ee_accel       = zeros(numDataSamples,6);

for i = 1:numDataSamples
    ee_vel(i, :) = dQ(i, :)*transpose(Jacob_schunk(Q(i, :)));
    T = FK_schunk(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);  % measured end-effector velocity
end

dt = diff(t);

for i=2:numDataSamples-1
    ee_accel(i,1:3) = (ee_vel(i+1,1:3) - ee_vel(i-1,1:3)) ./ (2*dt(i));  % calculated end-effector acceleration; Central diff
end

% based on Schunk datasheet:
Qdot_lim  = 72*pi/180*ones(6,1);  % rad/s
Qddot_lim = 150*pi/180*ones(6,1);  % rad/s^2
Xdot = Jacob_schunk(Qdot_lim)*Qdot_lim;  % Extreme cartesian velocity
Xddot = dJ_dt_fun(Qdot_lim, Qddot_lim)*Qdot_lim + Jacob_schunk(Qdot_lim)*Qddot_lim;  % Extreme cartesian acceleration


