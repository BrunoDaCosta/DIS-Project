%%
clear all; close all; clc;

%% Setup the Import Options
opts = delimitedTextImportOptions("NumVariables", 12);
% Specify range and delimiter
opts.DataLines = [3, inf];
opts.Delimiter = ";";
% Specify column names and types
opts.VariableNames = ["time", "fitness", "orientation", "distance", "velocity"];
opts.VariableTypes = ["double", "double", "double", "double", "double" ];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
% Import the data
metrics_01_iter_1 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\basic config ku = 0.4, kw = 0.3\flocking_metrics_iter_1.csv", opts);
metrics_01_iter_2 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\basic config ku = 0.4, kw = 0.3\flocking_metrics_iter_2.csv", opts);
metrics_01_iter_3 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\basic config ku = 0.4, kw = 0.3\flocking_metrics_iter_3.csv", opts);

nb_iter = 3;
nb_fit = 4;

mean_Fitness = zeros(1,nb_iter);
mean_Orientation = zeros(1,nb_iter);
mean_Distance = zeros(1,nb_iter);
mean_Velocity = zeros(1,nb_iter);

std_Fitness = zeros(1,nb_iter);
std_Orientation = zeros(1,nb_iter);
std_Distance = zeros(1,nb_iter);
std_Velocity = zeros(1,nb_iter);

saturation_perf = zeros(nb_iter,nb_fit);


clear opts

%%
close all;
datametrics = table2array(metrics_01_iter_1);
datametrics = datametrics(1:end-1,:);

figure;
hold on;
a1 = plot(datametrics(:,1), datametrics(:,2));
M1 = "Fitness";
a2 = plot(datametrics(:,1), datametrics(:,3));
M2 = "Orientation between robots";
a3 = plot(datametrics(:,1), datametrics(:,4));
M3 = "Distance between robots";
a4 = plot(datametrics(:,1), datametrics(:,5));
M4 = "Velocity of the team ";
legend([a1,a2,a3,a4], [M1, M2, M3, M4]);
title("Flocking metrics with initial configuration");
xlabel("Time [s]");
ylabel("Accuracy []");
hold off

