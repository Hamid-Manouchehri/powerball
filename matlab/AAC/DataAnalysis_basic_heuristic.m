clc;        close all;          clear

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO

%%% Loading data: time, EndEff Position, EndEff Rotation, EndEff Vel, F/T
dataset = fcn_load_csv(fName);
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
    ee_vel(i, :) = dQ(i, :)*transpose(fcn_Jacob_schunk(Q(i, :)));
    T = fcn_FK_schunk(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end

dt = diff(t);

for i=2:numDataSamples-1
    ee_accel(i,1:3) = (ee_vel(i+1,1:3) - ee_vel(i-1,1:3)) ./ (2*dt(i));  % Central diff
end

M_admit = 0.3*diag([1, 1, 1]);
C_admit = 10*diag([1.2, 1.0, 1.0]);

% figure;
% plot(t, ee_accel(:,1));

difference = zeros(numDataSamples,1);
for j=1:numDataSamples
    
    difference(j) = norm(FT(j,1:3)' - M_admit*ee_accel(j,1:3)' - C_admit*ee_vel(j,1:3)');

end

plot(t,difference)

figure;
plot(ee_pos(:, 2), ee_pos(:,1), 'b.');
hold on
xlabel("y (m)"); ylabel("x (m)");

epsilon = difference > 15;  % TODO
plot(ee_pos(epsilon == 1, 2), ee_pos(epsilon == 1,1), 'ro');