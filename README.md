# <span style="color: Darkorange;">SmartGate - Project for the course of Robotic Perception and Action</span> 

## <span style="color: orange;">Explanation of Matlab files given by Alessandro Luchetti</span> 

---
---

### 1. Main file:
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
  <img src="/Project_Material/Images/Original_Cloud_Points.jpg" />
</p>

<p align="center">
  <img src="/Project_Material/Images/Transformed_Cloud_Points.jpg" />
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
