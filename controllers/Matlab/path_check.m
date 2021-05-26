%%
clear all; close all; clc;

%% Setup the Import Options
opts = delimitedTextImportOptions("NumVariables", 12);
% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ";";
% Specify column names and types
opts.VariableNames = ["time", "pose_x", "pose_y", "pose_heading", "gps_x", "gps_y", "speed_x", "speed_y", "acc_x", "acc_y", "actual_pos_x", "actual_pos_y"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
% Import the data
test1 = readtable("C:\Users\Jérémy\Documents\EPFL\MA-2\Distr_Intel_Sys\project\DIS-Project\controllers\localization_controller\odoenc.csv", opts);
clear opts

%%
close all;
data = table2array(test1);
plot(data(:,2), data(:,3));
hold on 
plot(data(:,5), data(:,6));
plot(data(:,11), data(:,12));
hold off 