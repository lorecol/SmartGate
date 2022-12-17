clc
clear all
close all

%%

mainFolder = 'Calibration20220922_14_23_47_135';
cloud0 =  pcread( fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread( fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread( fullfile(mainFolder,'018408745047cloud0.ply'));

% fullfile create a path of the specified file in the specified folder
% pcread reads the point cloud from the specified .ply file returning a
% pointCloud object

% pointCloud object:
%   - Location: position of the points in 3D coordinate space
%   - Color: set the color of points in point cloud
%   - Normal: specify the normal vector with respect to each point in the point cloud
%   - Intensity: grayscale intensities at each point
%   - Count: total number of points in the point cloud
%   - XLimits, YLimits, ZLimits: range of coordinates along the 3 axes

H0 = load('H0.txt'); % roto-translation matrix of frontal camera
H1 = load('H1.txt'); % roto-translation matrix of left lateral camera
H2 = load('H2.txt'); % roto-translation matrix of right lateral camera

Hnorm = [ 0 1 0 0 ; 0 0 1 0 ; 1 0 0 0 ; 0 0 0 1 ]

points0 = [cloud0.Location'; ones(1,length(cloud0.Location))]; % create a 4x4 matrix where the first 3 rows
%                                                                are the (x,y,z) coordinates of the points 
%                                                                in the point cloud 0, while the last row 
%                                                                is full of ones
points1 = [cloud1.Location'; ones(1,length(cloud1.Location))]; % same as above but for point cloud 1
points2 = [cloud2.Location'; ones(1,length(cloud2.Location))]; % same as above but for point cloud 2

points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;

% Plot points in 3D space from the three cloud points
figure(1), clf, hold on, grid on, axis equal
plot3( cloud0.Location(:,1), cloud0.Location(:,2), cloud0.Location(:,3), '.r','markersize', 0.1)
plot3( cloud1.Location(:,1), cloud1.Location(:,2), cloud1.Location(:,3), '.g','markersize', 0.1)
plot3( cloud2.Location(:,1), cloud2.Location(:,2), cloud2.Location(:,3), '.b','markersize', 0.1)
title("Original points from the cloud points")
cameratoolbar % displays a camera toolbar in the current figure that enables interactive manipulation 
%               of the axes camera and light

figure(2), clf, hold on, grid on, axis equal
draw3dReferenceSystems()
plot3( points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r','markersize', 0.1)
plot3( points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g','markersize', 0.1)
plot3( points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b','markersize', 0.1)
title("Transformated points from the cloud points")

%% EQUALIZZA IMMAGINI

fileinfo = dir('Calibration20220922_14_25_02_597/*.png');  % get list of files in current directory
addpath("Calibration20220922_14_25_02_597")
thisfilename = fileinfo(1).name;  % current file name
figure(3)
I = imread(thisfilename); % reads the image from the specified file, inferring the format of the file 
%                           from its contents
I_eq = histeq(I*20/2.303);  % LE IMMAGINI VANNO EQUALIZZATE IN MANIERA LOGARITMICA COSI'

% histeq performs histogram equalization --> for example to enhance the constrast of an intensity image; 
% it transforms the grayscale image so that the histogram of the output grayscale image (I_eq) has 64 
% bins and is approximately flat

imshow(I_eq); % displays the grayscale image in a figure