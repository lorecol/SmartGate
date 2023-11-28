function [points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder)

%EVALCLOUDPOINTS This function reads and shows the original cloud points 
% coming from the 3 cameras. Then it pre-process them by removing the noise 
% and by performing downsampling, references them w.r.t. cam0 reference 
% frame and finally displays the transformed cloud points.

% Read the original cloud points
cloud0 =  pcread(fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread(fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread(fullfile(mainFolder,'018408745047cloud0.ply'));
 
% Remove the noise >> a point is considered to be an outlier if the average 
%                     distance to its k-nearest neighbors is above the 
%                     specified threshold
ptCloud0 = pcdenoise(cloud0, "Threshold", 0.5);
ptCloud1 = pcdenoise(cloud1, "Threshold", 0.5);
ptCloud2 = pcdenoise(cloud2, "Threshold", 0.5);

% Downsample the cloud points >>  downsampling is used to reduce the size 
%                                 of an input image before it is processed 
%                                 further. The resulting point cloud 
%                                 retains the overall geometric structure 
%                                 but has a reduced number of points
ptCloud0 = pcdownsample(ptCloud0, "nonuniformGridSample", 12);
ptCloud1 = pcdownsample(ptCloud1, "nonuniformGridSample", 12);
ptCloud2 = pcdownsample(ptCloud2, "nonuniformGridSample", 12);

% Load the transformation matrices associated to the 3 cameras
H0 = load('H0.txt'); 
H1 = load('H1.txt'); 
H2 = load('H2.txt'); 

Hnorm = [0 1 0 0 ; 0 0 1 0 ; 1 0 0 0 ; 0 0 0 1];

% Extract x/y/z-coordinates of the point clouds
points0 = [ptCloud0.Location'; ones(1,length(ptCloud0.Location))];
points1 = [ptCloud1.Location'; ones(1,length(ptCloud1.Location))]; 
points2 = [ptCloud2.Location'; ones(1,length(ptCloud2.Location))]; 

% Reference the points w.r.t. cam0 reference system
points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;

% Plot the original cloud points
figure, clf, hold on, grid on, axis equal
plot3(cloud0.Location(:,1), cloud0.Location(:,2), cloud0.Location(:,3), ...
    '.r','markersize', 0.1)
plot3(cloud1.Location(:,1), cloud1.Location(:,2), cloud1.Location(:,3), ...
    '.g','markersize', 0.1)
plot3(cloud2.Location(:,1), cloud2.Location(:,2), cloud2.Location(:,3), ...
    '.b','markersize', 0.1)
title("Original points from the cloud points")

% Displays a camera toolbar in the current figure that enables interactive
% manipulation of the axes camera and light
cameratoolbar;

% Plot pre-processed and transformed cloud points
figure, clf, hold on, grid on, axis equal
% Recall the created function
draw3dReferenceSystems();
plot3(points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r', ...
    'markersize', 0.1);
plot3(points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g', ...
    'markersize', 0.1);
plot3(points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b', ...
    'markersize', 0.1);
title("Transformed points from the cloud points");

end