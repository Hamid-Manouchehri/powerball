clear; clc; close all;

% Link lengths
a1 = 500;
a2 = 400;

% Time vector
t = linspace(0,2*pi,500);

% Preallocate
x = zeros(size(t));
y = zeros(size(t));
theta1 = zeros(size(t));
theta2 = zeros(size(t));

% Generate trajectory + joint angles
for i = 1:length(t)
    
    [x(i),y(i)] = Figure8_50447516(t(i));
    
    [theta1(i),theta2(i)] = ...
        InverseKinematics_50447516(x(i),y(i),a1,a2);
    
end

%% ===== Plot 1: Cartesian Path =====
figure;
plot(x,y,'LineWidth',2)
xlabel('X (mm)')
ylabel('Y (mm)')
title('Figure 8 Cartesian Trajectory')
axis equal
grid on

%% ===== Plot 2: Joint Angles vs Time =====
figure;
plot(t,theta1,'LineWidth',2); hold on
plot(t,theta2,'LineWidth',2);
xlabel('Time')
ylabel('Joint Angles (rad)')
legend('\theta_1','\theta_2')
title('Joint Angles vs Time')
grid on