clc; clear; close all;

csv_file_dir = '/home/hamid-tuf/projects/powerball/matlab/peg_in_hole_data/';
fName_0 = csv_file_dir + "exp2_stiff_0_ld.csv";
fName_0_segmented = csv_file_dir + "exp2_stiff_0_ld_segment.csv";

fName_100 = csv_file_dir + "exp2_stiff_1550_ld.csv";
fName_100_segmented = csv_file_dir + "exp2_stiff_1550_ld_segment.csv";

data_0 = readtable(fName_0);
data_0_segmented = readtable(fName_0_segmented);

data_100 = readtable(fName_100);
data_100_segmented = readtable(fName_100_segmented);

x0 = data_0.x;
y0 = data_0.y;
z0 = data_0.z;
xdot0 = data_0.xdot;
ydot0 = data_0.ydot;
zdot0 = data_0.zdot;
f0_x = data_0.FT1;
f0_y = data_0.FT2;
f0_z = data_0.FT3;

pks_idx0 = table2array(data_0_segmented(:,1));
valley_idx0 = table2array(data_0_segmented(:,2));
valley_idx0(3)=[];

pks_idx100 = table2array(data_100_segmented(:,1));
valley_idx100 = table2array(data_100_segmented(:,2));

x100 = data_100.x;
y100 = data_100.y;
z100 = data_100.z;
xdot100 = data_100.xdot;
ydot100 = data_100.ydot;
zdot100 = data_100.zdot;
f100_x = data_100.FT1;
f100_y = data_100.FT2;
f100_z = data_100.FT3;

figure;
subplot(3,1,1);
plot(f0_x); hold on;
plot(f100_x);
legend(["0 stiff", "100 stiff"]);

subplot(3,1,2);
plot(f0_y); hold on;
plot(f100_y);
legend(["0 stiff", "100 stiff"]);

subplot(3,1,3);
plot(f0_z); hold on;
plot(f100_z);
legend(["0 stiff", "100 stiff"]);

pks_idx100 = table2array(data_100_segmented(:,1));
valley_idx100 = table2array(data_100_segmented(:,2));
valley_idx100(3) = [];

c = 'rgbcmy';

for ins=1:size(data_100_segmented,1)

    j=1;
    for smpl=pks_idx100(ins):valley_idx100(ins)
        jamming_idx(ins, j) = f100_x(smpl) / f100_z(smpl);
        j = j + 1;
    end

end

% for i=1:1
%     e = valley_idx0(i);
%     s = e-500;
% 
%     % plot3(x,y,z,'.')
%     % hold on
%     % plot3(x(pks_idx),y(pks_idx),z(pks_idx),'o')
%     % plot3(x(valley_idx),y(valley_idx),z(valley_idx),'s')
%     % id =1:length(z);
%     % figure
%     % plot(id,z)
%     % hold on
%     % plot(id(valley_idx),z(valley_idx),'s')
%     % plot(id(pks_idx),z(pks_idx),'o')
%     xseg = x0(s:e)-x0(e);
%     yseg = y0(s:e)-y0(e);
%     zseg = z0(s:e)-z0(e);
%     xd = xdot0(s:e);
%     yd = ydot0(s:e);
%     zd = zdot0(s:e);
% 
%     figure(1)
%     plot(xseg,yseg); hold on
%     r = sqrt(xseg.^2 + yseg.^2);
% 
%     figure(2)
%     plot(r,zseg,[c(i) 'o']); hold on
%     rd = diff(r);
%     rd = [rd; 0];
% 
%     figure(3)
%     plot(xseg,xd,[c(i) ':']); hold on
% 
%     figure(4)
%     plot(yseg,yd,[c(i) ':']); hold on
% 
%     figure(5)
%     plot(zseg,zd,[c(i) ':']); hold on
% 
% end
% 
% 
% for i=1:1
%     e = valley_idx100(i);
%     s = e-500;
% 
%     % plot3(x,y,z,'.')
%     % hold on
%     % plot3(x(pks_idx),y(pks_idx),z(pks_idx),'o')
%     % plot3(x(valley_idx),y(valley_idx),z(valley_idx),'s')
%     % id =1:length(z);
%     % figure
%     % plot(id,z)
%     % hold on
%     % plot(id(valley_idx),z(valley_idx),'s')
%     % plot(id(pks_idx),z(pks_idx),'o')
%     xseg = x100(s:e)-x100(e);
%     yseg = y100(s:e)-y100(e);
%     zseg = z100(s:e)-z100(e);
%     xd = xdot100(s:e);
%     yd = ydot100(s:e);
%     zd = zdot100(s:e);
% 
%     figure(6)
%     plot(xseg,yseg); hold on
%     r = sqrt(xseg.^2 + yseg.^2);
% 
%     figure(7)
%     plot(r,zseg,[c(i) 'o']); hold on
%     rd = diff(r);
%     rd = [rd; 0];
% 
%     figure(8)
%     plot(xseg,xd,[c(i) ':']); hold on
% 
%     figure(9)
%     plot(yseg,yd,[c(i) ':']); hold on
% 
%     figure(10)
%     plot(zseg,zd,[c(i) ':']); hold on
% 
% end




