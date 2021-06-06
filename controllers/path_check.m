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
acc = readtable("C:\EPFL\DIS-Project\controllers\results_localization\odo_only_acc.csv", opts);
enc = readtable("C:\EPFL\DIS-Project\controllers\results_localization\odo_only_enc.csv", opts);
accgps = readtable("C:\EPFL\DIS-Project\controllers\results_localization\odo_gps_acc.csv", opts);
encgps = readtable("C:\EPFL\DIS-Project\controllers\results_localization\odo_gps_enc.csv", opts);
sup = readtable("C:\EPFL\DIS-Project\controllers\results_localization\super.csv", opts);

clear opts

%%
close all;
dataacc = table2array(acc);
erroracc = 0;

dataenc = table2array(enc);
errorenc = 0;

dataaccgps = table2array(accgps);
erroraccgps = 0;

dataencgps = table2array(encgps);
errorencgps = 0;
errorgps = 0;

datasup = table2array(sup);

max_index = 7188;
truc=zeros(max_index,5);
for i=1:1:max_index
    truc(i,1) = sqrt((datasup(i,2)-dataacc(i,2))^2 + (datasup(i,3)-dataacc(i,3))^2);
    erroracc = erroracc+ truc(i,1);
    
    truc(i,2) = sqrt((datasup(i,2)-dataenc(i,2))^2 + (datasup(i,3)-dataenc(i,3))^2);
    errorenc = errorenc + truc(i,2);
    
    truc(i,3) = sqrt((datasup(i,2)-dataaccgps(i,2))^2 + (datasup(i,3)-dataaccgps(i,3))^2);
    erroraccgps = erroraccgps + truc(i,3);
    
    truc(i,4) = sqrt((datasup(i,2)-dataencgps(i,2))^2 + (datasup(i,3)-dataencgps(i,3))^2);
    errorencgps = errorencgps + truc(i,4);

    truc(i,5) = sqrt((datasup(i,2)-dataencgps(i,5))^2 + (datasup(i,3)-dataencgps(i,6))^2);
    if(i>63)
        errorgps = errorgps + truc(i,5);
    end 
end

mean_error=sum(truc,1)/max_index

figure;
hold on;
a1 = plot(dataacc(:,2), dataacc(:,3));
M1 = "Acceleration odometry";
a2 = plot(dataenc(:,2), dataenc(:,3));
M2 = "Encoder odometry";
a3 = plot(dataaccgps(:,2), dataaccgps(:,3));
M3 = "Acceleration odometry + GPS";
a4 = plot(dataencgps(:,2), dataencgps(:,3));
M4 = "Encoder odometry + GPS";
a5 = plot(datasup(:,2), datasup(:,3));
M5 = "Supervisor";
a6 = plot(dataencgps(63:end,5), dataencgps(63:end,6));
M6 = "gps";
legend([a1,a2,a3,a4,a5,a6], [M1, M2, M3, M4, M5, M6]);


hold off