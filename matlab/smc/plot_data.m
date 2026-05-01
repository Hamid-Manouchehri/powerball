clc; clear; close all;

files = dir('./hand_drawing/data/*.mat');

for k = 1:length(files)
    
    

    % Display current file name
    fprintf('Loading: %s\n', files(k).name);
    
    % Load the file and pause for user input
    load(files(k).name);
    
    % --- Display/Plot data here ---
    % Example: plot(dataVariable); 
    
    % Wait for user to press enter
    input('Press Enter to continue to next file...', 's');
end
