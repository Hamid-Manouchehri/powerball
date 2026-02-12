clc;        close all;          clear;

dataDirName = '/home/hamid-tuf/projects/powerball/data/admittance_control/test_4_021126/';
fName_damp10 = [dataDirName 'hm_test_damp_10_damp_10.000000_01.csv'];  % TODO
fName_damp100 = [dataDirName 'hm_test_damp_100_damp_100.000000_01.csv'];  % TODO

fcn_DataAnalysis_improved_heuristic(fName_damp10);
fcn_DataAnalysis_improved_heuristic(fName_damp100);