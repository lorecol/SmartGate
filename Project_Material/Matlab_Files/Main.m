clc
clear all
close all

%% 

% There is a script that allows the user to choose which of the 2 provided 
% dataset to evaluate.
% After the choice, first the images provided by the 3 cameras are
% equalized, and then the point clouds are evaluated and displayed in a
% graphical way

addpath(genpath(pwd))

%% CHOOSE THE DATASET TO STUDY

[datasets, mainFolder] = DatasetChoice();

%% EQUALIZE CAMERA IMAGES

fileinfo = EqualizeCamImage(mainFolder);

%% EVALUATE CLOUD POINTS

EvalCloudPoints(mainFolder);

%% DATA FILTERING

% TODO
