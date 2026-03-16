clear; clc; close all;

read_schunk_file = ["lowDamp_10_schunk_mat.mat", ...
                    "midDamp_50_schunk_mat.mat", ...
                    "highDamp_100_schunk_mat.mat"];

dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/mat/";

read_schunk_file = dir + read_schunk_file;
fName = cell(3,1);
% a = load(read_schunk_file(1));
for i=1:3
    fName{i} = load(read_schunk_file(i));
end

fName