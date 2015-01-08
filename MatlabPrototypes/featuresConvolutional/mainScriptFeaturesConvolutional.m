initScript;
load([dataDirectory 'dataset.mat']);
% loadRawData;

voxelSide = 0.1;
filterSide = 3; % odd and >= 3

buildVoxelCardinalityMap;