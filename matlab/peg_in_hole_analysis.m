clc; clear; close all;

csv_file_dir = '/home/hamid-tuf/projects/powerball/matlab/peg_in_hole_data/';
fName = csv_file_dir + "exp2_stiff_0_ld.csv";
fName_segmented = csv_file_dir + "exp2_stiff_0_ld_segment.csv";

data = readtable(fName);
data_segmented = readtable(fName_segmented);
x = data.x;
y = data.y;
z = data.z;
xdot = data.xdot;
ydot = data.ydot;
zdot = data.zdot;

pks_idx = table2array(data_segmented(:,1));
valley_idx = table2array(data_segmented(:,2));
valley_idx(3)=[];
c = 'rgbkmy';
for i=1:1
    e = valley_idx(i);
    s= e-500;
    
    % plot3(x,y,z,'.')
    % hold on
    % plot3(x(pks_idx),y(pks_idx),z(pks_idx),'o')
    % plot3(x(valley_idx),y(valley_idx),z(valley_idx),'s')
    % id =1:length(z);
    % figure
    % plot(id,z)
    % hold on
    % plot(id(valley_idx),z(valley_idx),'s')
    % plot(id(pks_idx),z(pks_idx),'o')
    xseg = x(s:e)-x(e);
    yseg = y(s:e)-y(e);
    zseg = z(s:e)-z(e);
    xd = xdot(s:e);
    yd = ydot(s:e);
    zd = zdot(s:e);
    
    figure(1)
    plot(xseg,yseg); hold on
    r = sqrt(xseg.^2+yseg.^2);
    
    figure(2)
    plot(r,zseg,[c(i) 'o']); hold on
    rd = diff(r);
    rd = [rd; 0];
    
    figure(3)
    plot(xseg,xd,[c(i) ':']); hold on
    
    figure(4)
    plot(yseg,yd,[c(i) ':']); hold on
    
    figure(5)
    plot(zseg,zd,[c(i) ':']); hold on

end

% plot(z(pks_idx(1):))

% 
% 
% figure;
% subplot(3,1,1);
% plot(x);
% ylabel("x");
% 
% subplot(3,1,2)
% plot(y);
% ylabel("y");
% 
% subplot(3,1,3);
% plot(z);
% ylabel("z");
% % hold on;
% yline(mean(z), 'r--', 'Threshold'); % simplest way for horizontal line
% % hold off;
% 
% outDir  = fullfile(pwd, 'segmented');
% mkdir(outDir);
% files = dir(fullfile(csv_file_dir, '*.csv'));
% for j = 1:length(files)
%     fName = fullfile(files(j).folder, files(j).name);
%     data  = readtable(fName);
%     z = data.z;
%     [valley_val, valley_idx] = findpeaks(-z, 'MinPeakProminence', 0.03);
%     major_valleys_val = valley_val;
%     major_valleys_idx = valley_idx;
%     [peak_val, peak_idx] = findpeaks(z, 'MinPeakProminence', 0.02);
%     major_peaks_val = peak_val;
%     major_peaks_idx = peak_idx;
%     major_peaks_idx   = major_peaks_idx(z(major_peaks_idx) > mean(z));
%     major_valleys_idx = major_valleys_idx(z(major_valleys_idx) < mean(z));
%     segments = [];
%     for i = 1:length(major_peaks_idx)
%         next_valley = major_valleys_idx(major_valleys_idx > major_peaks_idx(i));
%         if isempty(next_valley)
%             break;
%         end
%         next_valley = next_valley(1);
%         segments(end+1, :) = [major_peaks_idx(i), next_valley(1)];
%     end
%     segments = array2table(segments, 'VariableNames', {'peak_idx', 'valley_idx'});
%     [~, name, ~] = fileparts(files(j).name);
%     outName = fullfile(outDir, [name '_segment.csv']);
%     writetable(segments, outName);
% end



