clc
clear all
close all

%% TO EQUALIZE IMAGES

fileinfo = dir('Calibration20220922_14_25_02_597/*.png');  % get list of files in current directory
addpath("Calibration20220922_14_25_02_597")
thisfilename = struct('Image1', fileinfo(1).name, 'Image2', fileinfo(2).name, 'Image3', fileinfo(3).name);
images = fieldnames(thisfilename);

for i = 1:numel(images)
    figure(i)
    imshow(histeq((imread(fileinfo(i).name))*20/2.303))
end

% imread( ) reads the image from the specified file, inferring the format of the file from its contents

% histeq( ) performs histogram equalization --> for example to enhance the constrast of an intensity image; 
% it transforms the grayscale image so that the histogram of the output grayscale image (I_eq) has 64 
% bins and is approximately flat

% imshow( ) displays the grayscale image in a figure

%% MAIN

mainFolder = 'Calibration20220922_14_23_47_135';
cloud0 =  pcread( fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread( fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread( fullfile(mainFolder,'018408745047cloud0.ply'));

% fullfile create a path of the specified file in the specified folder

H0 = load('H0.txt'); 
H1 = load('H1.txt'); 
H2 = load('H2.txt'); 

Hnorm = [ 0 1 0 0 ; 0 0 1 0 ; 1 0 0 0 ; 0 0 0 1 ]

points0 = [cloud0.Location'; ones(1,length(cloud0.Location))];
points1 = [cloud1.Location'; ones(1,length(cloud1.Location))]; 
points2 = [cloud2.Location'; ones(1,length(cloud2.Location))]; 

points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;

% Plot points in 3D space from the three cloud points
figure(4), clf, hold on, grid on, axis equal
plot3( cloud0.Location(:,1), cloud0.Location(:,2), cloud0.Location(:,3), '.r','markersize', 0.1)
plot3( cloud1.Location(:,1), cloud1.Location(:,2), cloud1.Location(:,3), '.g','markersize', 0.1)
plot3( cloud2.Location(:,1), cloud2.Location(:,2), cloud2.Location(:,3), '.b','markersize', 0.1)
title("Original points from the cloud points")
cameratoolbar % displays a camera toolbar in the current figure that enables interactive manipulation 
%               of the axes camera and light

figure(5), clf, hold on, grid on, axis equal
% Recall the created function
draw3dReferenceSystems()
plot3( points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r','markersize', 0.1)
plot3( points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g','markersize', 0.1)
plot3( points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b','markersize', 0.1)
title("Transformated points from the cloud points")