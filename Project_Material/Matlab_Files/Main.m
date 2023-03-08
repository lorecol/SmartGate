clc
clear all
close all

%% MAIN
% There is a script that allows the user to choose which of the 2 provided 
% dataset to evaluate.
% After the choice, first the images provided by the 3 cameras are
% equalized, and then the point clouds are evaluated and displayed in a
% graphical way

% Datasets to choose from. They have to be inside Datasets.
% If necessary append folder name
datasets = [
    "Datasets/Calibration20220922_14_23_47_135", ...
    "Datasets/Calibration20220922_14_25_02_597"
    ]; 
addpath(genpath(pwd))

% Choose from which dataset import. 
data_choice = input("Which data set you want to evaluate?\n - 1 for the 1st" + ...
    " dataset\n - 2 for the 2nd dataset\n\n Choice: ");

if data_choice==1
    mainFolder=datasets(data_choice);
elseif data_choice==2
    mainFolder=datasets(data_choice);
else
    error('Dataset not allowed')
end
addpath(mainFolder);

%% EQUALIZE CAMERA IMAGES

% get list of png files in current directory
fileinfo = dir(mainFolder+"/*.png");  

% Creating a struct of images name
for k = 1:length(fileinfo)
    imagesname.('Image'+string(k)) = fileinfo(k).name;
end

% Show equalized images
cam = [0 1 2];
for i = 1:length(fileinfo)
    figure
    imshow(histeq((imread(fileinfo(i).name))*20/2.303))
    title("Cam" + cam(i))
end

% histeq performs histogram equalization --> for example to enhance the 
% constrast of an intensity image; 
% it transforms the grayscale image so that the histogram of the output 
% grayscale image (I_eq) has 64 bins and is approximately flat

%% Evaluate cloud points
cloud0 =  pcread(fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread(fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread(fullfile(mainFolder,'018408745047cloud0.ply'));

% fullfile create a path of the specified file in the specified folder

H0 = load('H0.txt'); 
H1 = load('H1.txt'); 
H2 = load('H2.txt'); 

Hnorm = [ 0 1 0 0 ; 0 0 1 0 ; 1 0 0 0 ; 0 0 0 1 ];

points0 = [cloud0.Location'; ones(1,length(cloud0.Location))];
points1 = [cloud1.Location'; ones(1,length(cloud1.Location))]; 
points2 = [cloud2.Location'; ones(1,length(cloud2.Location))]; 

points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;

% Plot points in 3D space from the three cloud points
figure, clf, hold on, grid on, axis equal
plot3( cloud0.Location(:,1), cloud0.Location(:,2), cloud0.Location(:,3), '.r','markersize', 0.1)
plot3( cloud1.Location(:,1), cloud1.Location(:,2), cloud1.Location(:,3), '.g','markersize', 0.1)
plot3( cloud2.Location(:,1), cloud2.Location(:,2), cloud2.Location(:,3), '.b','markersize', 0.1)
title("Original points from the cloud points")

% displays a camera toolbar in the current figure that enables interactive
% manipulation of the axes camera and light
cameratoolbar

figure, clf, hold on, grid on, axis equal
% Recall the created function
draw3dReferenceSystems()
plot3( points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r','markersize', 0.1)
plot3( points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g','markersize', 0.1)
plot3( points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b','markersize', 0.1)
title("Transformated points from the cloud points")

