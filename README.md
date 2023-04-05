# <span style="color: Darkorange;">SmartGate - Project for the course of Robotic Perception and Action</span> 

## <span style="color: orange;">Explanation of Matlab files</span> 

---
---

### Main file:
```Matlab
%% INITIALIZATION

clc
clear all
close all

% There is a script that allows the user to choose which of the 2 provided 
% dataset to evaluate.
% After the choice, first the images provided by the 3 cameras are
% equalized, then the point clouds are evaluated and displayed in a
% graphical way, and finally they are filtered

addpath(genpath(pwd))

% Choose the dataset to study
[datasets, mainFolder] = DatasetChoice();

%% EQUALIZE CAMERA IMAGES

fileinfo = EqualizeCamImage(mainFolder);

%% EVALUATE CLOUD POINTS

[points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder);

%% DATA FILTERING

PtCloudFilt = DataFiltering(points0rt, points1rt, points2rt);

% Save the filtered point cloud into a binary encoded ply file
pcwrite(PtCloudFilt, "Project_Material/Filtered_Point_Cloud.ply", ...
    Encoding = "binary");

clear all; clc;
```

**_Functions:_**

 - **_DatasetChoice()_** allows the user to choose which dataset evaluate. It returns:
   - datasets: the path of all datasets
   - mainFolder: the path of the chosen dataset

 - **_fileinfo = EqualizeCamImage(mainFolder)_** equalizes the images provided by the three cameras and display them. It returns:
   - fileinfo: a structure with, among other things, the name of the images

 - **_[points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder)_** reads the cloud points from the chosen dataset, performs system transformation in order to reference all w.r.t. camera 0 and displays the original and transformed cloud points. It returns:
   - points0rt, points1rt, points2rt: the points provided by the three cameras that have been referenced w.r.t. camera 0

 - **_PtCloudFilt = DataFiltering(points0rt, points1rt, points2rt)_** reads the transformed points and recalls a function named **_CutPoints()_** that filters the point cloud. It also plots the filered point cloud. It returns:
   - PtCloudFilt: filtered point cloud  
 
 - **_[remainPtCloud, IndexPtCluster] = CutPoints(points0rt, points1rt, points2rt)_** filters the point cloud fitting the unwanted planes such as the ground to the point cloud. Furthermore, it completes the filtering collecting the points in clusters and removing the unwanted clusters. It returns:
   - remainPtCloud: point cloud after plane fitting filtering
   - IndexPtCluster: struct containing the clusters of points

 - **_draw3dReferenceSystems(transformationMatrixToRS, name, scale, width)_** is used in order to add in a 3D plot a cartesian reference system defined by the three axes x, y and z.

---
---
#### 1. **_DatasetChoice_** function:
```Matlab
function [datasets, mainFolder] = DatasetChoice()

% This function allows the user to choose from the available datasets. The
% chosen dataset is then processed

% Datasets to choose from. They have to be inside the "Datasets" folder
datasets = [
    "Project_Material/Datasets/Calibration20220922_14_23_47_135", ...
    "Project_Material/Datasets/Calibration20220922_14_25_02_597"
    ]; 

% Choose from which dataset import the data
data_choice = input("Which data set you want to evaluate?\n - 1 for the 1st" + ...
    " dataset\n - 2 for the 2nd dataset\n\n Choice: ");

% Allow the choice
if data_choice == 1
    mainFolder = datasets(data_choice);
elseif data_choice == 2
    mainFolder = datasets(data_choice);
else
    error('Dataset not allowed')
end

end
```
---
---
#### 2. **_EqualizeCamImage_** function:
```Matlab
function fileinfo = EqualizeCamImage(mainFolder)

% This function equalizes the images provided by the three ToF cameras to 
% allow them to be viewed

% Get the list of all png files in current directory
fileinfo = dir(mainFolder+"/*.png");  

% Show equalized images
cam = [0 1 2];
for i = 1:length(fileinfo)
    figure
    imshow(histeq((imread(fileinfo(i).name))*20/2.303))
    title("Cam" + cam(i))
end

% histeq performs histogram equalization (e.g. to enhance the constrast of 
% an intensity image); it transforms the grayscale image so that the 
% histogram of the output grayscale image has 64 bins and is 
% approximately flat

end
```
---
---
#### 3. **_EvalCloudPoints_** function:
```Matlab
function [points0rt, points1rt, points2rt] = EvalCloudPoints(mainFolder)

% This function reads and shows the original cloud points coming from the 3
% cameras. Then it pre-process them (removing the noise and downsampling),
% references them w.r.t. cam0 frame and finally displays the transformed 
% cloud points

% Read the original cloud points
cloud0 =  pcread(fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread(fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread(fullfile(mainFolder,'018408745047cloud0.ply'));
 
% Remove the noise
ptCloud0 = pcdenoise(cloud0, "Threshold", 0.5);
ptCloud1 = pcdenoise(cloud1, "Threshold", 0.5);
ptCloud2 = pcdenoise(cloud2, "Threshold", 0.5);
% Downsample the cloud points
ptCloud0 = pcdownsample(ptCloud0, "nonuniformGridSample", 12);
ptCloud1 = pcdownsample(ptCloud1, "nonuniformGridSample", 12);
ptCloud2 = pcdownsample(ptCloud2, "nonuniformGridSample", 12);

% Load the transformation matrices associated to the 3 cameras
H0 = load('H0.txt'); 
H1 = load('H1.txt'); 
H2 = load('H2.txt'); 

Hnorm = [0 1 0 0 ; 0 0 1 0 ; 1 0 0 0 ; 0 0 0 1];

points0 = [ptCloud0.Location'; ones(1,length(ptCloud0.Location))];
points1 = [ptCloud1.Location'; ones(1,length(ptCloud1.Location))]; 
points2 = [ptCloud2.Location'; ones(1,length(ptCloud2.Location))]; 

% Reference points w.r.t. cam0 system
points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;

% Plot original cloud points
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
cameratoolbar

% Plot pre-processed and transformed cloud points
figure, clf, hold on, grid on, axis equal
% Recall the created function
draw3dReferenceSystems()
plot3(points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r', ...
    'markersize', 0.1)
plot3(points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g', ...
    'markersize', 0.1)
plot3(points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b', ...
    'markersize', 0.1)
title("Transformed points from the cloud points")

end
```
---
---
#### 4. **_dataFiltering_** function:
```Matlab
function PtCloudFilt = DataFiltering(points0rt, points1rt, points2rt)

% This function recalls "CutPoints" function (which cut the cloud points)
% and plots the new cloud points, thus highlighting the figure of the 
% pallet

% Transpose to have the coordinates of the points as columns
points0rt = transpose(points0rt);
points1rt = transpose(points1rt);
points2rt = transpose(points2rt);

% Cut the points to highlight the pallet object - METHOD 1
[remainPtCloud, IndexPtCluster] = CutPoints(points0rt, points1rt, ...
    points2rt);

% Define filtered point cloud
PtCloudFilt = select(remainPtCloud, IndexPtCluster(1).Indexes);

% Remaining point cloud after having canceled unwanted portions
cameratoolbar
draw3dReferenceSystems()
figure, clf, hold on, grid on, axis equal
pcshow(PtCloudFilt)
axis on
xlabel('X');
ylabel('Y');
zlabel('Z');
title("Filtered points from the cloud points")

end
```
---
---
#### 5. **_CutPoints_** function:
```Matlab
function [remainPtCloud, IndexPtCluster] = CutPoints(points0rt, ...
    points1rt, points2rt)

% This function fits the unwanted planes of points (such as the ground) to 
% the point cloud. In this way we are able to cut these planes.
% Furthermore, the function divides the points into clusters of points in
% order to complete the filtering operation 

% Create point cloud objects from transformed points
points0rtCloud = pointCloud(points0rt(:, 1:3), 'Color', 'red');
points1rtCloud = pointCloud(points1rt(:, 1:3), 'Color', 'green');
points2rtCloud = pointCloud(points2rt(:, 1:3), 'Color', 'blue');

% Concatenate the point clouds
ptClouds = [points0rtCloud; points1rtCloud; points2rtCloud];
ptCloudOut = pccat(ptClouds);

% Rotate the point cloud to improve the point of view
rotationAngles = [90 -90 0];
translation = [0 0 0];
tform = rigidtform3d(rotationAngles, translation);
ptCloudOut = pctransform(ptCloudOut, tform);

% Fit the planes to remove (e.g. ground)
maxDistance1 = 0.05;
maxDistance2 = 0.5;
referenceVector = [0, 0, 1];
maxAngularDistance = 5;

[~, inlierIndices, outlierIndices] = pcfitplane(ptCloudOut, ...
            maxDistance1, referenceVector, maxAngularDistance);
select(ptCloudOut, inlierIndices);
remainPtCloud = select(ptCloudOut, outlierIndices);

roi = [-inf,inf; -1,inf; -inf,inf];
sampleIndices = findPointsInROI(remainPtCloud, roi);

[~, inlierIndices, outlierIndices] = pcfitplane(remainPtCloud, ...
            maxDistance2, SampleIndices = sampleIndices);
select(remainPtCloud, inlierIndices);
remainPtCloud = select(remainPtCloud, outlierIndices);

% Segment the point cloud in clusters
MinDistance = 0.7;
[labels, numClusters] = pcsegdist(remainPtCloud, MinDistance);

IndexPtCluster = struct;

for i = 1:numClusters
    IndexPtCluster(i).Indexes = find(labels == i);
end

end
```
---
---
#### 6. **_draw3dReferenceSystems_** function:
```Matlab
function draw3dReferenceSystems(transformationMatrixToRS, name, scale, width)

try % execute statement
    transformationMatrixToRS;
catch % catch resulting errors
    transformationMatrixToRS = eye(4); % 4x4 identity matrix
end

try
    name;
catch
    name = 'reference frame';
end

try
    scale;
catch
    scale = 1 ;
end

try
    width;
catch
    width = 3;
end

hold on
originPoint = transformationMatrixToRS * [ 0 0 0 1 ]';

xDirVector  = ( transformationMatrixToRS * [ 1 0 0 1 ]' - originPoint).* scale;
yDirVector  = ( transformationMatrixToRS * [ 0 1 0 1 ]'  - originPoint).* scale;
zDirVector  = ( transformationMatrixToRS * [ 0 0 1 1 ]' - originPoint).* scale;


px = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , xDirVector(1) , xDirVector(2) , xDirVector(3) , 'r' , 'LineWidth', width);
py = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , yDirVector(1) , yDirVector(2) , yDirVector(3) , 'g' , 'LineWidth', width);
pz = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , zDirVector(1) , zDirVector(2) , zDirVector(3) , 'b' , 'LineWidth', width);

% quiver3(X,Y,Z,U,V,W) plots arrows with directional components U, V, and W at the Cartesian coordinates 
% specified by X, Y, and Z

text( originPoint(1)+xDirVector(1) , originPoint(2)+xDirVector(2) , originPoint(3)+xDirVector(3) , 'x' );
text( originPoint(1)+yDirVector(1) , originPoint(2)+yDirVector(2) , originPoint(3)+yDirVector(3) , 'y' );
text( originPoint(1)+zDirVector(1) , originPoint(2)+zDirVector(2) , originPoint(3)+zDirVector(3) , 'z' );

text( originPoint(1) , originPoint(2) , originPoint(3) , name );
```
---
---
## <span style="color: orange;">Configure Unity to interact with HoloLens</span> 

[Introduction to the Mixed Reality Toolkit-Set Up Your Project and Use Hand Interaction](https://learn.microsoft.com/en-us/training/modules/learn-mrtk-tutorials/)