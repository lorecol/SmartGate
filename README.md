# <span style="color: Darkorange;">SmartGate - Project for the course of Robotic Perception and Action</span> 

## <span style="color: orange;">Explanation of Matlab files given by Alessandro Luchetti</span> 

---

### Main file:
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

---


function