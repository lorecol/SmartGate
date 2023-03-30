function remainPtCloud = CutPoints(points0rt, points1rt, points2rt)

% This function fits the unwanted planes of points (such as the floor) to 
% the point cloud. In this way we are able to cut these planes and keep 
% only the meaningful points, which are the points that define the shape of
% the pallet

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

maxDistance1 = 0.05;
maxDistance2 = 0.5;
referenceVector = [0, 0, 1];
maxAngularDistance = 5;

[model1, inlierIndices, outlierIndices] = pcfitplane(ptCloudOut, ...
            maxDistance1, referenceVector, maxAngularDistance);
select(ptCloudOut, inlierIndices);
remainPtCloud = select(ptCloudOut, outlierIndices);

roi = [-inf,inf; -1,inf; -inf,inf];
sampleIndices = findPointsInROI(remainPtCloud, roi);

[model2, inlierIndices, outlierIndices] = pcfitplane(remainPtCloud, ...
            maxDistance2, SampleIndices = sampleIndices);
select(remainPtCloud, inlierIndices);
remainPtCloud = select(remainPtCloud, outlierIndices);

end