clc; clear; close all;

v = 10:30;
windowSize = 6;
overlap = 2;
step = windowSize - overlap;

for i = 1:step:length(v)-windowSize+1
    window = v(i:i+windowSize-1);
    disp(window);
end