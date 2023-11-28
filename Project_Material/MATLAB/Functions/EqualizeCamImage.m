function fileinfo = EqualizeCamImage(mainFolder)

%EQUALIZECAMIMAGE This function equalizes the images provided by the three 
% ToF cameras to allow them to be viewed. It uses logarithmic
% transformation, which is a non-linear basic grey level transformation
% function which maps a narrow range of low grey values to a wider range of
% output levels and thus enhancing the contrast levels and brightness of
% the image.

% Get the list of all png files in current directory
fileinfo = dir(mainFolder + "/*.png");  

% Show equalized images
cam = [0 1 2];              % Number of ToF cameras
for i = 1:length(fileinfo)
    figure;
    imshow(histeq((imread(fileinfo(i).name)) * (20 / 2.303)));
    title("Cam" + cam(i));
end

% histeq() performs histogram equalization (e.g. to enhance the constrast 
% of an intensity image); it transforms the grayscale image so that the 
% histogram of the output grayscale image has 64 bins and is 
% approximately flat.

end