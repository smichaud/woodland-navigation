datasetDir = '../../../WoodlandNavigationData/SimulatedPointClouds/';

dataset = load([datasetDir 'dataset.mat'])
% dataset = loadSimulationDataset(datasetDir);
% save([datasetDir 'dataset.mat'], 'dataset');

trainRatio = 0.7;
testRatio = 1-trainRatio;