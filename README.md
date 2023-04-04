# <span style="color: Darkorange;">SmartGate - Project for the course of Robotic Perception and Action</span> 

## <span style="color: orange;">Explanation of Matlab files</span> 

---
TO BE UPDATED

---

### Main file:
```Matlab
%% CHOOSE THE DATASET TO STUDY

[datasets, mainFolder] = DatasetChoice();

%% EQUALIZE CAMERA IMAGES

fileinfo = EqualizeCamImage(mainFolder);

%% EVALUATE CLOUD POINTS

EvalCloudPoints(mainFolder);
```

**_Functions:_**

 - **_DatasetChoice()_** allows the user to choose which dataset evaluate. It returns:
   - datasets: the path of all datasets
   - mainFolder: the path of the chosen dataset

 - **_EqualizeCamImage(mainFolder)_** equalizes the images provided by the three cameras and display them. It returns:
   - fileinfo: a structure with, among other things, the name of the images

 - **_EvalCloudPoints(mainFolder)_** reads the cloud points from the chosen dataset, performs system transformation and display the original and transformed cloud points.
 
 - **_draw3dReferenceSystems(transformationMatrixToRS, name, scale, width)_** is used in order to add in a 3D plot a cartesian reference system defined by the three axes x, y and z.

---
---
#### 1. **_DatasetChoice_** function:
```Matlab
function [datasets, mainFolder] = DatasetChoice()

% Datasets to choose from. They have to be inside the "Datasets" folder
datasets = [
    "Project_Material/Datasets/Calibration20220922_14_23_47_135", ...
    "Project_Material/Datasets/Calibration20220922_14_25_02_597"
    ]; 

% Choose from which dataset import the data
data_choice = input("Which data set you want to evaluate?\n - 1 for the 1st" + ...
    " dataset\n - 2 for the 2nd dataset\n\n Choice: ");

if data_choice==1
    mainFolder=datasets(data_choice);
elseif data_choice==2
    mainFolder=datasets(data_choice);
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

% Get the list of all png files in current directory
fileinfo = dir(mainFolder+"/*.png");  

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
% grayscale image has 64 bins and is approximately flat

end
```
---
---
#### 3. **_EvalCloudPoints_** function:
```Matlab
function EvalCloudPoints(mainFolder)

cloud0 =  pcread(fullfile(mainFolder,'004373465147cloud0.ply'));
cloud1 =  pcread(fullfile(mainFolder,'007086770647cloud0.ply'));
cloud2 =  pcread(fullfile(mainFolder,'018408745047cloud0.ply'));

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

% Assign the variables to the "main" file workspace
assignin('base', 'cloud0', cloud0);
assignin('base', 'cloud1', cloud1);
assignin('base', 'cloud2', cloud2);
assignin('base', 'points0', points0);
assignin('base', 'points1', points1);
assignin('base', 'points2', points2);
assignin('base', 'points0rt', points0rt);
assignin('base', 'points1rt', points1rt);
assignin('base', 'points2rt', points0rt);

% Displays a camera toolbar in the current figure that enables interactive
% manipulation of the axes camera and light
cameratoolbar

figure, clf, hold on, grid on, axis equal
% Recall the created function
draw3dReferenceSystems()
plot3( points0rt(1,:), points0rt(2,:), points0rt(3,:), '.r','markersize', 0.1)
plot3( points1rt(1,:), points1rt(2,:), points1rt(3,:), '.g','markersize', 0.1)
plot3( points2rt(1,:), points2rt(2,:), points2rt(3,:), '.b','markersize', 0.1)
title("Transformed points from the cloud points")

end
```
---
---
#### 4. **_draw3dReferenceSystems_** function:
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

<!---
```Matlab
cloud0 =  pcread(fullfile(mainFolder, '004373465147cloud0.ply'));
cloud1 =  pcread(fullfile(mainFolder, '007086770647cloud0.ply'));
cloud2 =  pcread(fullfile(mainFolder, '018408745047cloud0.ply'));
``` 
This allows to read the point cloud from the specified .ply file returning a pointCloud object, which is composed by:
   - Location: position of the points in 3D coordinate space
   - Color: set the color of points in point cloud
   - Normal: specify the normal vector with respect to each point in the point cloud
   - Intensity: grayscale intensities at each point
   - Count: total number of points in the point cloud
   - XLimits, YLimits, ZLimits: range of coordinates along the 3 axes

```Matlab
H0 = load('H0.txt'); 
H1 = load('H1.txt'); 
H2 = load('H2.txt'); 
```
H0, H1, H2 are the roto-translation matrices of the three cameras:
   - H0 &rarr; frontal camera
   - H1 &rarr; left lateral camera
   - H2 &rarr; right lateral camera

```Matlab
points0 = [cloud0.Location'; ones(1, length(cloud0.Location))];
points1 = [cloud1.Location'; ones(1,length(cloud1.Location))]; 
points2 = [cloud2.Location'; ones(1,length(cloud2.Location))]; 
```
This creates a 4x4 matrix where the first 3 rows are respectively the x, y and z coordinates of the points in the specified point cloud, while the last row is full of ones

```Matlab
points0rt = points0;
points1rt = inv(H0) * H1 * points1;
points2rt = inv(H0) * H2 * points2;
```
This is done to transform the cloud points by means of roto-translation transformations

<p align="center">
  <img src="/Project_Material/Images/Dataset_Calibration20220922_14_23_47_135/Original_Cloud_Points.jpg" />
</p>

<p align="center">
  <img src="/Project_Material/Images/Dataset_Calibration20220922_14_23_47_135/Transformed_Cloud_Points.jpg" />
</p>

---
---

### 2. Function ``` draw3dReferenceSystems( transformationMatrixToRS , name , scale, width ) ```:

This function is used in order to add in a 3D plot a cartesian reference system defined by the three axes x, y and z

```Matlab
try 
    transformationMatrixToRS;
catch 
    transformationMatrixToRS = eye(4); 
end
```
This defines **transformationMatrixToRS** as a 4x4 identity matrix

```Matlab
try
    name;
catch
    name = 'reference frame';
end
```
This defines the **name** of the reference system as **reference frame**

```Matlab
try
    scale;
catch
    scale = 1 ;
end
```
This sets the **scale** of the reference system to **1**

```Matlab
try
    width;
catch
    width = 3;
end
```
This sets the **width** of the arrow that define the three axes of the ref. system to **3**

```Matlab
originPoint = transformationMatrixToRS * [ 0 0 0 1 ]';

xDirVector  = ( transformationMatrixToRS * [ 1 0 0 1 ]' - originPoint).* scale;
yDirVector  = ( transformationMatrixToRS * [ 0 1 0 1 ]'  - originPoint).* scale;
zDirVector  = ( transformationMatrixToRS * [ 0 0 1 1 ]' - originPoint).* scale;
```
This specifies the directions in which the three cartesian axes x, y, z are defined:
   - x &rarr; {1, 0, 0}
   - y &rarr; {0, 1, 0}
   - z &rarr; {0, 0, 1}

```Matlab
px = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , xDirVector(1) , xDirVector(2) , xDirVector(3) , 'r' , 'LineWidth', width);
py = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , yDirVector(1) , yDirVector(2) , yDirVector(3) , 'g' , 'LineWidth', width);
pz = quiver3( originPoint(1) , originPoint(2) , originPoint(3) , zDirVector(1) , zDirVector(2) , zDirVector(3) , 'b' , 'LineWidth', width);
```
In general **_quiver3(X, Y, Z, U, V, W)_** plots arrows with directional components U, V, and W at the Cartesian coordinates specified by X, Y, and Z
-->
