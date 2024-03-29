function [] = draw3dReferenceSystems( transformationMatrixToRS , name , scale, width )

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