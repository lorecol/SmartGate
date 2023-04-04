function DataFiltering(points0rt, points1rt, points2rt)

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

% Remaining point cloud after having canceled unwanted portions
cameratoolbar
draw3dReferenceSystems()
figure, clf, hold on, grid on, axis equal
pcshow(select(remainPtCloud, IndexPtCluster(1).Indexes))
axis on
xlabel('X');
ylabel('Y');
zlabel('Z');
title("Filtered points from the cloud points")

end