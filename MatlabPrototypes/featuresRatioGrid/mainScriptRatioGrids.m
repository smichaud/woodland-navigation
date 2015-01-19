initScript;
load([dataDirectory 'dataset.mat']);
% loadRawData;

nbOfSamples = length(dataset);

% Voxel Cardinality map creation ==========================================
voxelSide = 0.1; % in meters
if mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide)
    warning('Voxel side size does not fit into the area of interest');
end

ratioGridSide = 3; % odd and >= 3
if mod(ratioGridSide,2) ~= 1 || ratioGridSide < 3
    error('The ratioGrid side must be odd and greater or equal to 3')
end
nbOfPaddingVoxel = round((ratioGridSide - 1)/2);

nbOfX = round(areaOfInterest.depth/voxelSide) + 2*nbOfPaddingVoxel;
nbOfY = round(areaOfInterest.width/voxelSide) + 2*nbOfPaddingVoxel;
nbOfZ = round(areaOfInterest.height/voxelSide) + 2*nbOfPaddingVoxel;

buildVoxelCardinalityMap;

% RatioGrid creation ======================================================
nbOfRatioGridsPerSample = (nbOfX-2*nbOfPaddingVoxel)*...
    (nbOfY-2*nbOfPaddingVoxel)*(nbOfZ-2*nbOfPaddingVoxel);
totalNbOfRatioGrids = nbOfRatioGridsPerSample*nbOfSamples;

ratioGrids = zeros(totalNbOfRatioGrids, ratioGridSide^3);
buildRatioGridList;

% Clustering ==============================================================
nbOfClusters = 3;
distanceUsed = 'sqEuclidean'; % or 'cityblock'
manuallyCreateEmptyRatioGridCenter = true;

if manuallyCreateEmptyRatioGridCenter
    ratioGrids(find(sum(ratioGrids,2)==0),:) = [];
    nbOfClusters = nbOfClusters - 1;
end
[clustersIndices clusterCenters] = kmeans(...
    ratioGrids, nbOfClusters, 'distance', distanceUsed);
if manuallyCreateEmptyRatioGridCenter
    clusterCenters = [zeros(1,ratioGridSide^3) ; clusterCenters];
end
%==========================================================================
