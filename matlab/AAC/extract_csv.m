function [t, ee_pos, ee_rot, ee_vel, FT] = extract_csv(csvFile)

    opts = detectImportOptions(csvFile, 'Delimiter', ',');
    opts.VariableNamingRule = "preserve";   % keep original header text
    tsCol = opts.VariableNames{contains(opts.VariableNames, "TimeStamp")};
    opts = setvartype(opts, tsCol, "string");
    opts = setvaropts(opts, tsCol, 'WhitespaceRule', 'trim');
    
    data = readtable(csvFile, opts);

    
    t = datetime(data.TimeStamp, 'InputFormat', 'HH:mm:ss:SSSSSS');
    t = seconds(t - t(1));
    Q = data{:, {'Q1','Q2','Q3','Q4','Q5','Q6'}};
    dQ = data{:, {'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6'}};
    FT = data{:, {'FT1','FT2','FT3','FT4','FT5','FT6'}};
    
    %FT_xy_amp = sqrt(FT(:,1).^2 + FT(:,2).^2);
    
    numRowQ = size(Q, 1);
    ee_rot = zeros(numRowQ,3);
    ee_pos = zeros(numRowQ,3);
    ee_vel = zeros(numRowQ,6);
    
    for i = 1:numRowQ
        ee_vel(i, :) = dQ(i, :)*transpose(Jacob_schunk(Q(i, :)));
        T = FK_schunk(Q(i, :));
        T = T';
        ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
        ee_pos(i, :) = T(4, :);
    end
end