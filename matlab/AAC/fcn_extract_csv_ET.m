function [t, ee_pos, ee_rot, ee_vel, FT] = fcn_extract_csv_ET(fName)

    buf = importdata(fName);
    t_txt = buf.textdata(:,1);          L = length(t_txt);
    t = zeros(L-1,1);
    for i=2:L
        H = str2double(t_txt{i}(1:2));
        M = str2double(t_txt{i}(4:5));
        S = str2double(t_txt{i}(7:8));
        ss = str2double(t_txt{i}(10:end-1));
        t(i-1) = 3600*H+60*M+S+ss/1e6;
    end
    data = [t buf.data];
    
    Q = data(:, 2:7);
    dQ = data(:,8:13);
    FT = data(:,14:19);
        
    numRowQ = size(Q, 1);
    ee_rot = zeros(numRowQ,3);
    ee_pos = zeros(numRowQ,3);
    ee_vel = zeros(numRowQ,6);
    
    for i = 1:numRowQ
        ee_vel(i, :) = dQ(i, :)*transpose(fcn_Jacob_schunk(Q(i, :)));
        T = fcn_FK_schunk(Q(i, :));
        T = T';
        ee_rot(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
        ee_pos(i, :) = T(4, :);
    end
end