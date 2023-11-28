%MAIN There is a script that allows the user to choose which of the 2 provided 
% dataset to evaluate.
% After the choice, first the images provided by the 3 cameras are
% equalized, then the point clouds are evaluated and displayed in a
% graphical way, and finally they are filtered.
% Furthermore, it is possible to calibrate the camera with images taken
% with the camera itself.

%% INITIALIZATION

clearvars; close all; clc;

% Add all files and folders to Matlab path
addpath(genpath(pwd));

%% LOAD THE DATASET

[datasets, mainFolder] = DatasetChoice();

%% EQUALIZE CAMERA IMAGES

fileinfo = EqualizeCamImage(mainFolder);

%% EVALUATE CLOUD POINTS

[points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder);

%% DATA FILTERING

PtCloudFilt = DataFiltering(points0rt, points1rt, points2rt);

% Save the filtered point cloud into a binary encoded ply file
pcwrite(PtCloudFilt, "Project_Material/Filtered_PointCloud/Filtered_Point_Cloud.ply", ...
    Encoding = "binary");

%% CAMERA CALIBRATION

% Define images to process
imageFileNames = {...
    'Project_Material\Calibration\Images\20230406_072551_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_072811_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_072829_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073111_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073127_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073139_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073156_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073212_HoloLens.jpg', ...
    'Project_Material\Calibration\Images\20230406_073225_HoloLens.jpg', ...
    };

% Extract intrinsic and extrinsic camera parameters
[cameraParams, imagesUsed, estimationErrors] = CameraCalibration(imageFileNames);