initScript;
load([dataDirectory 'dataset.mat']);
% loadRawData;

nbOfSamples = length(dataset);

% Voxel Cardinality map creation ==========================================
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

% Filters creation ========================================================
nbOfFilterPerSample = (nbOfX-2*nbOfPaddingVoxel)*...
    (nbOfY-2*nbOfPaddingVoxel)*(nbOfZ-2*nbOfPaddingVoxel);
totalNbOfFilter = nbOfFilterPerSample*nbOfSamples;

filters = zeros(totalNbOfFilter, filterSide^3);
buildRatioGridList;

% Clustering ==============================================================
nbOfClusters = 3;
distanceUsed = 'sqEuclidean'; % or 'cityblock'
manuallyCreateEmptyRatioGrid = true;

if manuallyCreateEmptyRatioGrid
    filters(find(sum(filters,2)==0),:) = [];
    nbOfClusters = nbOfClusters - 1;
end
[clustersIndices clusterCenters] = kmeans(...
    filters, nbOfClusters, 'distance', distanceUsed);
if manuallyCreateEmptyRatioGrid
    clusterCenters = [zeros(1,filterSide^3) ; clusterCenters];
end
%==========================================================================
