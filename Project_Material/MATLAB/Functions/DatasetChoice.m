function [datasets, mainFolder] = DatasetChoice()

%DATASETCHOICE This function allows the user to choose which dataset to 
% load from all the available datasets. When the dataset is chosen, three
% ply files containing the point cloud taken by the ToF cameras are loaded.

% Datasets to choose from. They have to be inside the "Datasets" folder
datasets = [
            "Project_Material/Datasets/Calibration20220922_14_23_47_135", ...
            "Project_Material/Datasets/Calibration20220922_14_25_02_597"
           ]; 

% Choose from which dataset import the data
data_choice = input("Which data set you want to evaluate?\n - 1 for the 1st" + ...
                    " dataset\n - 2 for the 2nd dataset\n\n Choice: "             );

% Allow the choice
if data_choice == 1
    mainFolder = datasets(data_choice);
elseif data_choice == 2
    mainFolder = datasets(data_choice);
else
    error('Dataset not allowed!');
end

end