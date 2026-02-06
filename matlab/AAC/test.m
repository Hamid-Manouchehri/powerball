% C = {2, "Hamid", {[1,2,3], 'a'}, 1.23}
clc; clear; close all;
function dataset = loaddata(Filename)

    offset = 0;
    data = readtable(Filename, 'Format', 'auto');
    
    time = table2array(data( (1+offset):(end-offset),  1    ));
    q    = table2array(data( (1+offset):(end-offset),  2:7  ));
    qd   = table2array(data( (1+offset):(end-offset),  8:13 ));
    FT   = table2array(data( (1+offset):(end-offset), 14:19 ));
    
    timesteps = zeros(length(time), 1);
    
    for i = 1:length(time)
        timesteps(i,1) = sum(sscanf(string(time(i)), '%f:%f:%f:%f') .* [3600; 60; 1; 10^-6]);
        % timesteps(i,1) = sum(sscanf(string(time(i)), '%f:%f') .* [1; 10^-6]);
    end
    
    timesteps = timesteps - timesteps(1,:);
    
    for i = 1:length(time)
        if timesteps(i,:) < 0
            timesteps(i,:) = timesteps(i,:) + 60;
        end
    end
    
    dataset.timesteps  = timesteps;
    dataset.q          = q;
    dataset.qd         = qd;
    dataset.FT         = FT;

end


dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_2_020426/';
fName = [dataDirName 'et_maze_damp_10.000000_01.csv'];  % TODO

data = loaddata(fName, 0);