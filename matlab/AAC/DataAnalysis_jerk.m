clc;        close all;          clear

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO
C = 10;

%%% Loading data: time, EndEff Position, EndEff Rotation, EndEff Vel, F/T
dataset = fcn_load_csv(fName);
t = dataset.timesteps;
Q = dataset.q;
dQ = dataset.qd;
FT = dataset.FT;
admit_vel = dataset.vel;

%%% change to cartesian-space
numDataSamples = size(Q, 1);
ee_pos         = zeros(numDataSamples,3);
ee_rot         = zeros(numDataSamples,3);
ee_vel         = zeros(numDataSamples,6);

for i = 1:numDataSamples
    ee_vel(i, :) = dQ(i, :)*transpose(fcn_Jacob_schunk(Q(i, :)));
    T = fcn_FK_schunk(Q(i, :));
    T = T';
    ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
    ee_pos(i, :) = T(4, :);
end


%%% numerical computation of jerk
jEE = ee_pos(5:end,:)-2*ee_pos(4:end-1,:)+2*ee_pos(2:end-3,:)-ee_pos(1:end-4,:);
dt = diff(t);

jEE = [zeros(3);jEE./repmat(dt(3:end-1),1,3);zeros(1,3)];
JerkAmp = sqrt(sum(jEE.^2,2));
plot(JerkAmp)
id = JerkAmp>0.015;  % Thresholding
plot(ee_pos(:,2),0.5-ee_pos(:,1),'b.')
hold on
plot(ee_pos(id==1,2),0.5-ee_pos(id==1,1),'or')
% return

figure(2)
for i=100:length(id)
    cla
    if id(i)==1
        plot(ee_pos(i-99:i,2),0.5-ee_pos(i-99:i,1),'r','LineWidth',2); hold on
        plot(ee_pos(i,2),0.5-ee_pos(i,1),'ro','MarkerSize',8,'MarkerFaceColor','r')
        pause(0.1)
    else
        plot(ee_pos(i-99:i,2),0.5-ee_pos(i-99:i,1),'b','LineWidth',2); hold on

        plot(ee_pos(i,2),0.5-ee_pos(i,1),'bo','MarkerSize',8,'MarkerFaceColor','b')
        pause(0.001)
    end
    xlim([-0.1 0.2])
    ylim([0 0.2])
    
end
