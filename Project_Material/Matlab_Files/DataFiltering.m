function DataFiltering(points0rt, points1rt, points2rt)

% This function recalls "CutPoints" function ( which cut the cloud points)
% and plots the new cloud points, thus highlighting the figure of the 
% pallet

% Transpose to have the coordinates of the points as columns
points0rt = transpose(points0rt);
points1rt = transpose(points1rt);
points2rt = transpose(points2rt);

% Cut the points to highlight the pallet object
[points0, points1, points2] = CutPoints(points0rt, points1rt, points2rt); 

% Plot the cut cloud points
cameratoolbar
draw3dReferenceSystems()
figure, clf, hold on, grid on, axis equal
plot3(points0(:, 1), points0(:, 2), points0(:, 3), '.r','markersize', 0.1)
plot3(points1(:, 1), points1(:, 2), points1(:, 3), '.g','markersize', 0.1)
plot3(points2(:, 1), points2(:, 2), points2(:, 3), '.b','markersize', 0.1)
title("Filtered points from the cloud points")

end