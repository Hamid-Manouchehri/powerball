clc; clear;

% 1) Static symbols (no q(t))
syms q1 q2 dq1 dq2 real

% 2) Matrix A(q)
A = [ cos(q1),              sin(q2);
      cos(q2)*cos(q1),      sin(q1) ];

% 3) Chain rule using dq1,dq2
dA_dt = diff(A,q1)*dq1 + diff(A,q2)*dq2;

% 4) Save as a function file
matlabFunction(dA_dt, ...
    'File','dA_dt_fun', ...
    'Vars',{[q1;q2],[dq1;dq2]});

% Example call:
dA = dA_dt_fun([0.5;1.0],[0.2;-0.1]);
