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

% Rotate the point cloud in the original configuration
% rotationAngles = [-90 90 0];
% translation = [0 0 0];
% tform = rigidtform3d(rotationAngles, translation);
% PtCloudFilt = pctransform(PtCloudFilt, tform);
rotationAngles = [0 0 90];
translation = [0 0 0];
tform = rigidtform3d(rotationAngles, translation);
PtCloudFilt = pctransform(PtCloudFilt, tform);

% Remaining point cloud after having canceled unwanted portions
cameratoolbar
figure, clf, hold on, grid on, axis equal
draw3dReferenceSystems()
pcshow(PtCloudFilt)
axis on
xlabel('X');
ylabel('Y');
zlabel('Z');
title("\color{black}Filtered points from the cloud points")
set(gcf, 'color', 'w');
set(gca, 'color', 'w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])

end