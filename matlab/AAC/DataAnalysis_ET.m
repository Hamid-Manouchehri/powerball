clc;        close all;          clear all

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName = [dataDirName 'et_maze_damp_10_damp_10.000000_01.csv'];  % TODO
C = 10;

%%% Loading data: time, EndEff Position, EndEff Rotation, EndEff Vel, F/T
[t, X, Q, Xd, F] = fcn_extract_csv_ET(fName);
plot(t,F,'.')

jEE = X(5:end,:)-2*X(4:end-1,:)+2*X(2:end-3,:)-X(1:end-4,:);
dt = diff(t);

jEE = [zeros(3);jEE./repmat(dt(3:end-1),1,3);zeros(1,3)];
JerkAmp = sqrt(sum(jEE.^2,2));
plot(JerkAmp)
id = JerkAmp>0.15;
plot(X(:,2),0.5-X(:,1),'b.')
hold on
plot(X(id==1,2),0.5-X(id==1,1),'ro')
return

figure(2)
for i=100:length(id)
    cla
    if id(i)==1
        plot(X(i-99:i,2),0.5-X(i-99:i,1),'r','LineWidth',2); hold on
        plot(X(i,2),0.5-X(i,1),'ro','MarkerSize',8,'MarkerFaceColor','r')
        pause(0.1)
    else
        plot(X(i-99:i,2),0.5-X(i-99:i,1),'b','LineWidth',2); hold on

        plot(X(i,2),0.5-X(i,1),'bo','MarkerSize',8,'MarkerFaceColor','b')
        pause(0.001)
    end
    xlim([-0.1 0.2])
    ylim([0 0.2])
    

end
