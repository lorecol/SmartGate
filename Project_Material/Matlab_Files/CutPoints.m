function [points0, points1, points2] = CutPoints(points0rt, points1rt, points2rt)

% This function cuts the x, y and z-points that do not satisfy the
% boundaries imposed. In this way we are able to keep only the meaningful
% points, which are the points that define the shape of the pallet

% Cut x-points
points0rtx = points0rt(:, 1);
points1rtx = points1rt(:, 1);
points2rtx = points2rt(:, 1);
points0rtx(points0rtx < -0.55) = -1;
points1rtx(points1rtx < -0.55) = -1;
points2rtx(points2rtx < -0.55) = -1;

% Cut y-points
points0rty = points0rt(:, 2);
points1rty = points1rt(:, 2);
points2rty = points2rt(:, 2);
points0rty(points0rty < -0.7 | points0rty > 0.3) = 2;
points1rty(points1rty < -0.7 | points1rty > 0.3) = 2;
points2rty(points2rty < -0.7 | points2rty > 0.3) = 2;

% Cut z-points
points0rtz = points0rt(:, 3);
points1rtz = points1rt(:, 3);
points2rtz = points2rt(:, 3);
points0rtz(points0rtz < 1.3 | points0rtz > 2.3) = 0;
points1rtz(points1rtz < 1.3 | points1rtz > 2.3) = 0;
points2rtz(points2rtz < 1.3 | points2rtz > 2.3) = 0;

points0 = [points0rtx, points0rty, points0rtz];
points1 = [points1rtx, points1rty, points1rtz];
points2 = [points2rtx, points2rty, points2rtz];

end