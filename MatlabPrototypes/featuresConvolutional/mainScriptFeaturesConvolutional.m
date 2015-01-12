initScript;
load([dataDirectory 'dataset.mat']);
% loadRawData;

nbOfSamples = length(dataset);

voxelSide = 0.1;
if mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide)
    warning('Voxel side size does not fit into the area of interest');
end

filterSide = 3; % odd and >= 3
if mod(filterSide,2) ~= 1 || filterSide < 3
    error('The voxel filter must be odd and greater or equal to 3')
end
nbOfPaddingVoxel = round((filterSide - 1)/2);

nbOfX = round(areaOfInterest.depth/voxelSide) + 2*nbOfPaddingVoxel;
nbOfY = round(areaOfInterest.width/voxelSide) + 2*nbOfPaddingVoxel;
nbOfZ = round(areaOfInterest.height/voxelSide) + 2*nbOfPaddingVoxel;

buildVoxelCardinalityMap;
