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
metrics_01_iter_1 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\Flock size\flocking_metrics_5.csv", opts);
metrics_01_iter_2 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\Flock size\flocking_metrics_6.csv", opts);
metrics_01_iter_3 = readtable("C:\Users\maxim\Desktop\Master\MA2\Distributed intelligent systems\Projet\DIS-Project\controllers\metrics flocking\Flock size\flocking_metrics_7.csv", opts);

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
title("Flocking metrics with flock size = 5");
xlabel("Time [s]");
ylabel("Accuracy []");
hold off

% Mean and std for iter_1
mean_Fitness(1) = mean(datametrics(:,2));
mean_Orientation(1) = mean(datametrics(:,3));
mean_Distance(1) = mean(datametrics(:,4));
mean_Velocity(1) = mean(datametrics(:,5));

std_Fitness(1) = std(datametrics(:,2));
std_Orientation(1) = std(datametrics(:,3));
std_Distance(1) = std(datametrics(:,4));
std_Velocity(1) = std(datametrics(:,5));

saturarion_perf(1,:) = datametrics(end,2:5);

% Data iter_2
datametrics = table2array(metrics_01_iter_2);
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
title("Flocking metrics with flock size = 6");
xlabel("Time [s]");
ylabel("Accuracy []");
hold off

mean_Fitness(2) = mean(datametrics(:,2));
mean_Orientation(2) = mean(datametrics(:,3));
mean_Distance(2) = mean(datametrics(:,4));
mean_Velocity(2) = mean(datametrics(:,5));

std_Fitness(2) = std(datametrics(:,2));
std_Orientation(2) = std(datametrics(:,3));
std_Distance(2) = std(datametrics(:,4));
std_Velocity(2) = std(datametrics(:,5));

saturarion_perf(2,:) = datametrics(end,2:5);

% Data iter_2
datametrics = table2array(metrics_01_iter_3);
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
title("Flocking metrics with flock size = 7");
xlabel("Time [s]");
ylabel("Accuracy []");
hold off

mean_Fitness(3) = mean(datametrics(:,2));
mean_Orientation(3) = mean(datametrics(:,3));
mean_Distance(3) = mean(datametrics(:,4));
mean_Velocity(3) = mean(datametrics(:,5));

std_Fitness(3) = std(datametrics(:,2));
std_Orientation(3) = std(datametrics(:,3));
std_Distance(3) = std(datametrics(:,4));
std_Velocity(3) = std(datametrics(:,5));

saturarion_perf(3,:) = datametrics(end,2:5);

%% 
x = [5,6,7];
errorbar(x,mean_Fitness,std_Fitness);
xticks([5 6 7])
title("Mean and Standard deviation for overall fitness");
ylabel("Accuracy []");
xlabel("Flock size []");

%%

figure;

hold on;
x = [5,6,7];

a1 = plot(x, saturarion_perf(:,1));
M1 = "Fitness";
a2 = plot(x, saturarion_perf(:,2));
M2 = "Orientation between robots";
a3 = plot(x, saturarion_perf(:,3));
M3 = "Distance between robots";
a4 = plot(x, saturarion_perf(:,4));
M4 = "Velocity of the team ";
legend([a1,a2,a3,a4], [M1, M2, M3, M4]);
xticks([5 6 7]);
title("Flocking saturation metrics");
ylabel("Accuracy []");
xlabel("Flock size []");