clear all
close all
clc

datasetDir = '../../../WoodlandNavigationData/SimulatedPointClouds/';

isLoadedFromCsv = true;
if isLoadedFromCsv
    disp('Loading point couds from csv files...');
    dataset = loadSimulationDataset(datasetDir);
    save([datasetDir 'dataset.mat'], 'dataset');
else
    disp('Loading dataset from dataset.mat...');
    load([datasetDir 'dataset.mat']);
end

isDatasetFiltered = true;
if isDatasetFiltered
    disp('Remove noisy example and extra samples...');
    samplesCount = length(dataset);
    indexToRemove = [];
    for i = 1:samplesCount
        if dataset(i).isNoisy == true
            indexToRemove(end+1) = i;
        end
    end
    dataset(indexToRemove) = [];
    
    if length(dataset) > 15
        dataset(16:end) = [];
    end
end

disp('Creating the points count voxel grid for all samples')
voxelSize = 0.1;
areaOfInterest = [-1,1 ; -1,1 ; -1,1];
dataset = buildPointsCountVoxelGrids(...
    dataset, voxelSize, areaOfInterest);


disp('Creating a list of all existing words (aka ratioGrids/features)')
wordGridSize = 3;
[dataset completeWordList] = buildWordLists(dataset, wordGridSize);
