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