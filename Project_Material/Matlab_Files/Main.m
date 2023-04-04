clc
clear all
close all

%% 

% There is a script that allows the user to choose which of the 2 provided 
% dataset to evaluate.
% After the choice, first the images provided by the 3 cameras are
% equalized, then the point clouds are evaluated and displayed in a
% graphical way, and finally they are filtered

addpath(genpath(pwd))

%% CHOOSE THE DATASET TO STUDY

[datasets, mainFolder] = DatasetChoice();

%% EQUALIZE CAMERA IMAGES

fileinfo = EqualizeCamImage(mainFolder);

%% EVALUATE CLOUD POINTS

[points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder);

%% DATA FILTERING

DataFiltering(points0rt, points1rt, points2rt);

clear points0rt points1rt points2rt