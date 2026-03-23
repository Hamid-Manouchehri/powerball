function [p3, J3, F3] = Q1(theta1, theta2, d3, obj)

% Geometry (mm)
L1 = 500;
L2 = 500;
d1 = 400;

% End-effector position
p3 = [L1*cos(theta1) + L2*cos(theta1 + theta2);
      L1*sin(theta1) + L2*sin(theta1 + theta2);
      d1 + d3];

% Attractive force
zeta = 5e-2;
F3 = -zeta * (p3 - obj);

% Jacobian
J3 = [ -L1*sin(theta1) - L2*sin(theta1+theta2),  -L2*sin(theta1+theta2), 0;
        L1*cos(theta1) + L2*cos(theta1+theta2),   L2*cos(theta1+theta2), 0;
        0,                                        0,                     1];

end