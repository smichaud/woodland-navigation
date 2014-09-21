disp(['Loading data from ',datasetName, ' ...']);

filename = 'data.mat';
load(strcat(dataDirectory,filename));

nbOfSamples = size(dataset,1);