clc;        close all;          clear;

load('./data/admittance/IDETC26/mat/IDETC_2026_data_modified_2.mat')
% rows are subject
% columns d=10 50 100 HW
damping_names = ["gross", "gross2fine", "fine", "fine2gross"];
m = {zeros(4,4) , zeros(4,4) , zeros(4,4) , zeros(4,4)};
for s=1:4
    for d=1:4  % iterate over dampings
        m{s}(:,d) = mean(data(s,d).entropy, 2); % mean of entropy for each segment over all trials
    end

    figure(1)
    subplot(4,1,s)
    bar(damping_names, m{s})
    ylabel(sprintf("subject %s", num2str(s)));
    legend(["C=10", "C=50", "C=100", "hand-writing"]);
    ylim([0 3])

end