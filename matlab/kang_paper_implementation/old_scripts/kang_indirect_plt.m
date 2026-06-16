clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

save_mat_file = "test_indirect_schunk.mat";  % TODO
matDir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/admittance/mat/";


for i=4t:4

    read_mat_file = "indirect_" + num2str(i) + "_schunk.mat";
    read_mat_file = matDir + read_mat_file;
    load(read_mat_file);


    figure(Name="ee_pos",NumberTitle="off");


    speed = sqrt(v_meas(:,1).^2 + v_meas(:,2).^2);

    scatter(ee_pos(:,1), ee_pos(:,2), 20, speed, 'filled');
    xlabel("x");
    ylabel("y");
    axis equal;
    grid on;
    cb = colorbar;
    colormap jet;
    cb.Label.String = "speed (m/s)";
    title('Trajectory colored by speed');


    figure(Name="T_V_F", NumberTitle="off");
    subplot(2,1,1);
    plot(t, v_meas(:,1));
    hold on;
    plot(t, v_meas(:,2));
    ylabel("Velocity [m/s]");
    legend(["X", "Y"]);
    subplot(2,1,2);
    plot(t, F_cmd(:,1));
    hold on;
    plot(t, F_cmd(:,2));
    xlabel("Time [s]")
    ylabel("F_{cmd} [N]");
    legend(["X", "Y"]);

end