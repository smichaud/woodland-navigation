init; % clear all
dataDirectory = './data/';
defineDatasetStruct;

loadDataset; % if already loaded and saved

loadRawData; % stock raw data from a folder in the structure "dataset"

extractTraversabilityCost % add the label to the structure
extractAreaOfInterest % extract the 3D area to be traversed by the robot

showInfo; % Add image beside

saveDataset;


