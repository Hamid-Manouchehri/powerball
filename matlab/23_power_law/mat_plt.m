clc; clear; close all;

addpath("/home/hamid-tuf/projects/powerball/matlab/23_power_law/functions/")

hand_written_maze_dir = "/home/hamid-tuf/projects/powerball/matlab/23_power_law/data/";
hand_written_maze_mat = "hand_drawn_maze_1.mat";
hand_written_maze_mat = hand_written_maze_dir + hand_written_maze_mat;

hand_written_maze = load(hand_written_maze_mat);

x = hand_written_maze.pts(:, 1);
y = hand_written_maze.pts(:, 2);
t = hand_written_maze.pts(:, 3);

figure;
plot(x, y);
xlabel("x (m)");
ylabel("y (m)");
title("hand written maze.");
