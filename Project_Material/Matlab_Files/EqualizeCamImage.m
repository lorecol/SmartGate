function fileinfo = EqualizeCamImage(mainFolder)

% Get the list of all png files in current directory
fileinfo = dir(mainFolder+"/*.png");  

% Show equalized images
cam = [0 1 2];
for i = 1:length(fileinfo)
    figure
    imshow(histeq((imread(fileinfo(i).name))*20/2.303))
    title("Cam" + cam(i))
end

% histeq performs histogram equalization --> for example to enhance the 
% constrast of an intensity image; 
% it transforms the grayscale image so that the histogram of the output 
% grayscale image has 64 bins and is approximately flat

end