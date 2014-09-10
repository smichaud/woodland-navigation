init; % clear all
defineDataStruct;

% loadDataset; % if already loaded and saved

loadRawData; % stock raw data from a folder in the structure "dataset"
extractTraversabilityCost; % add the label to the structure
extractAreaOfInterest; % extract the 3D area to be traversed by the robot
extractFeatures; % 

showData; % Image, point cloud area of interest, currents/cost
showResults; % Feature/cost, regression
printLoop;

saveDataset;