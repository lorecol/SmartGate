function [remainPtCloud, IndexPtCluster] = CutPoints(points0rt, ...
    points1rt, points2rt)

%CUTPOINTS This function fits the unwanted planes of points (such as the 
% ground) to the point cloud. In this way we are able to cut these planes.
% Furthermore, the function divides the points into different clusters of 
% points so the isolated ones can be removed, thus completing the filtering 
% operation.

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
MinDistance = 0.7;                   % min euclidean distance between points from different clusters
[labels, numClusters] = pcsegdist(remainPtCloud, MinDistance);

IndexPtCluster = struct;
for i = 1:numClusters
    IndexPtCluster(i).Indexes = find(labels == i);
end

end