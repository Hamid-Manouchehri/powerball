clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/functions/");

read_csv_dir = "/home/hamid-tuf/projects/powerball/matlab/" + ...
    "kang_paper_implementation/data/kang_paper/";

schunk_data_file = "sub1/const_10_1_schunk.csv";  % TODO
read_csv_file = read_csv_dir + schunk_data_file;
schunk_data = readtable(read_csv_file);

BIG_CIRCLE_CIRCUMFERENCE = 0.3142;  % meter
SQUARE_CIRCUMFERENCE = 0.32;  % meter

t          = (schunk_data.Time_us - schunk_data.Time_us(1)) / 1e6;
Q          = schunk_data{:,2:7};    % Schunk angular pos
Qdot_a     = schunk_data{:,8:13};   % Schunk angular velocity

schunk_numOfDataSamples = size(t, 1);
ee_pos    = zeros(schunk_numOfDataSamples,3);
ee_vel    = zeros(schunk_numOfDataSamples,6);

% Extracting cartesian space data from schunk:
for i = 1:schunk_numOfDataSamples
    ee_vel(i, :) = Qdot_a(i, :) * transpose(Jacob_schunk_fun(Q(i, :)));
    T = FK_schunk_fun(Q(i, :));
    T = T';
    ee_pos(i, :) = T(4, :);
end

path_time_data_file = "sub1/sub1_damp_10.csv";
read_csv_file = read_csv_dir + path_time_data_file;
path_time_data = readtable(read_csv_file);

% path_time_data{1,:}
circle_time = path_time_data{1,1:2};
square_time = path_time_data{1,5:6};

% indices for circle and square time intervals
idx_circle = find(t >= circle_time(1) & t <= circle_time(2));
idx_square = find(t >= square_time(1) & t <= square_time(2));

% circle path length
dx_c = diff(ee_pos(idx_circle,1));
dy_c = diff(ee_pos(idx_circle,2));
big_circle_length = sum(sqrt(dx_c.^2 + dy_c.^2));
big_circle_length_error = big_circle_length - BIG_CIRCLE_CIRCUMFERENCE;

% square path length
dx_s = diff(ee_pos(idx_square,1));
dy_s = diff(ee_pos(idx_square,2));
square_length = sum(sqrt(dx_s.^2 + dy_s.^2));
square_length_error = square_length - SQUARE_CIRCUMFERENCE;

