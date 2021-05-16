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
acc = readtable("C:\EPFL\DIS-Project\controllers\Matlab\odoacc.csv", opts);
enc = readtable("C:\EPFL\DIS-Project\controllers\Matlab\odoenc.csv", opts);
sup = readtable("C:\EPFL\DIS-Project\controllers\Matlab\super.csv", opts);

clear opts

%%
close all;
dataacc = table2array(acc);
dataenc = table2array(enc);
datasup = table2array(sup);
figure;
hold on;
a1 = plot(dataacc(:,2), dataacc(:,3));
M1 = "Acceleration odometry";
a2 = plot(dataenc(:,2), dataenc(:,3));
M2 = "Encoder odometry";
a3 = plot(datasup(:,2), datasup(:,3));
M3 = "Supervisor";
legend([a1,a2,a3], [M1, M2, M3]);


% plot(data(:,5), data(:,6));
% plot(data(:,11), data(:,12));
hold off