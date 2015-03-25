clear all
close all
clc

datasetDir = '../../../WoodlandNavigationData/SimulatedPointClouds/';

isLoadedFromCsv = false;
if isLoadedFromCsv
    disp('Loading point couds from csv files...');
    dataset = loadSimulationDataset(datasetDir);
    save([datasetDir 'dataset.mat'], 'dataset');
    
    disp('Creating the points count voxel grid for all samples')
    voxelSize = 0.1;
    areaOfInterest = [-1,1 ; -1,1 ; -1,1];
    dataset = buildPointsCountVoxelGrids(...
        dataset, voxelSize, areaOfInterest);
else
    load([datasetDir 'dataset.mat']);
    
    isDatasetFiltered = true;
    if isDatasetFiltered
        samplesCount = length(dataset);
        indexToRemove = [];
        for i = 1:samplesCount
            if dataset(i).isNoisy == true || i > 30
                indexToRemove(end+1) = i;
            end
        end
        dataset(indexToRemove) = []  
    end
end

disp('Creating a list of all existing words (aka ratioGrids/features)')
