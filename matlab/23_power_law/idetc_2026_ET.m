clc;        close all;          clear;
load('./data/admittance/IDETC26/mat/IDETC_2026_data.mat')
% rows are subject
% columns d=10 50 100 HW
m = {zeros(4,4) , zeros(4,4) , zeros(4,4) , zeros(4,4)};
for s=2:4
    for d=1:4
        m{s}(:,d) = mean(data(s,d).entropy,2); % mean of entropy over each subject trials 
    end
    figure(1)
    subplot(4,1,s)
    bar(transpose(m{s}))
    ylabel(sprintf("subject %s", s));
    legend()
    ylim([0 3])

    figure(2)
    subplot(4,1,s)
    bar(m{s})
    ylabel(sprintf("subject %s", s));
    legend()
    ylim([0 3])


end