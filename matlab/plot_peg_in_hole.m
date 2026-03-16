clc; clear; close all;

csv_file_dir = '/home/hamid-tuf/projects/powerball/matlab/peg_in_hole_data/';

% peg_in_hole_data = readtable(csv_file);

% plot3(peg_in_hole_data.x, peg_in_hole_data.y, peg_in_hole_data.z, "ro");
% plot(peg_in_hole_data.z, "o")

filePattern = fullfile(csv_file_dir, '*.csv'); % Creates a full path pattern
csvFiles = dir(filePattern);


%{
for i = 1:length(csvFiles)
    baseFileName = csvFiles(i).name;
    
    data_file = fullfile(csv_file_dir, baseFileName);

    subplot(4,2,i);
    plot()

end
%}
stiffValue = [0 100 350 950 1550];
condition = {'ld','hd'};
for e = 1:3
    figure(e)
    cnt = 0;
    for c = 1:2
        for s=1:5
            cnt = cnt+1;
            fName = [csv_file_dir 'exp' num2str(e) '_stiff_' num2str(stiffValue(s)) '_' condition{c} '.csv'];
            if isfile(fName)
                peg_in_hole_data = readtable(fName);
                subplot(2,5,cnt)
                %plot(peg_in_hole_data.z)
                x = peg_in_hole_data.x;
                y = peg_in_hole_data.y;
                z = peg_in_hole_data.z;
                yd = peg_in_hole_data.xdot;
                plot3(x,y,z,'.')

            end

        end
    end
end

figure;
fName = csv_file_dir + "exp2_stiff_950_hd.csv";
peg_in_hole_data = readtable(fName);
%plot(peg_in_hole_data.z)
x = peg_in_hole_data.x;
y = peg_in_hole_data.y;
z = peg_in_hole_data.z;
zd = peg_in_hole_data.zdot;
plot3(x,y,z,'.')
plot(z,zd,'.')

figure;
fName = csv_file_dir + "exp2_stiff_950_hd.csv"
file = readtable(fName);
plot(file.z);
hold on
plot(file.y);