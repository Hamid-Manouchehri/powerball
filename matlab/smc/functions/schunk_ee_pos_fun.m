function [t, ee_pos, ee_orient, ee_vel] = schunk_ee_pos_fun(read_schunk_file)

    schunk_table = readtable(read_schunk_file);
    
    t    = (schunk_table.Time_us - schunk_table.Time_us(1)) / 1e6;
    Q    = schunk_table{:,2:7};
    Qdot = schunk_table{:,8:13};   % Schunk angular velocity
    
    schunk_numOfDataSamples = size(t, 1);
    
    ee_pos    = zeros(schunk_numOfDataSamples,3);
    ee_orient = zeros(schunk_numOfDataSamples,3);
    ee_vel    = zeros(schunk_numOfDataSamples,6);
    
    % Extracting cartesian space data from schunk:
    for i = 1:schunk_numOfDataSamples
        ee_vel(i, :) = Qdot(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
        T = FK_schunk_fun(Q(i, :));
        T = T';
        ee_orient(i, :) = rotm2eul(T(1:3,:), 'ZYX');  % [yaw pitch roll] in radians
        ee_pos(i, :) = T(4, :);
    end

end