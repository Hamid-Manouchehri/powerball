clc;    close all;      clear;

sName = {'hamid','jason','kg','tahsin'};
damp = [10 50 100];
dirName = '/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/IDETC26/mat/';

for s=1:4  % # subjects
    for i=1:5  % # trials per subjects
        for d = 1:4  % # cases per subjects: {hand writing, low-damp, mid-damp, high-damp}

            if d<4
                fName = [sName{s} '_exp' num2str(i) '_damp_' num2str(damp(d)) '_mat.mat'];
            else
                fName = [sName{s} '_exp' num2str(i) '_hw_mat.mat'];
            end

            load([dirName fName])  % [low-damp, mid-damp, high-damp, hw]
            data(s,d).winData{i} = [window_center_time; beta_hat; K_hat];
            data(s,d).posData{i} = [schunk_time_s;x;y;R];
            data(s,d).entropy(:,i) = entropy.zone_num;
            data(s,d).info = {'winData: t, Beta_hat, k_hat'; 'PointData: t, x, y, R'};

        end
    end
end

save('./data/admittance/IDETC26/mat/IDETC_2026_data.mat','data')
