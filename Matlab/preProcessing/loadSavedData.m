disp(['Loading data from ',datasetName, ' ...']);

filename = 'dataset.mat';
load(strcat(dataDirectory,filename));

nbOfSamples = size(dataset,1);