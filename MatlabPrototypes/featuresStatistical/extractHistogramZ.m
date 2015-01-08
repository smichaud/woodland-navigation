% Split the area of interest into voxel and create an histogram of nbOfPts
greatestCommonDivisor = ...
    gcd(areaOfInterest.depth*100,areaOfInterest.height*100);
voxelSide = greatestCommonDivisor/100;
nbOfX = round(areaOfInterest.depth/voxelSide);
nbOfY = round(areaOfInterest.width/voxelSide);
nbOfVoxels = nbOfX*nbOfY;

nbOfBins = 4;
maxPtsPerVoxel = 60; % Choose approximately...
ranges = 0:maxPtsPerVoxel/nbOfBins:maxPtsPerVoxel-maxPtsPerVoxel/nbOfBins;

nbOfSamples = length(dataset);
for sampleIndex=1:nbOfSamples
    columnNbOfPoints = zeros(1,nbOfVoxels);
    pointCloud = dataset(sampleIndex).areaOfInterest;
    
    % Create the voxel and save nb of pts
    globalIndex = 1;
    for i = 1:nbOfX
        minX = (i-1)*voxelSide + areaOfInterest.distFromRobot + ...
            areaOfInterest.xTfAdjustment;
        maxX = i*voxelSide + areaOfInterest.distFromRobot + ...
            areaOfInterest.xTfAdjustment;
        for j = 1:nbOfY
            minY = (j-1)*voxelSide - areaOfInterest.width/2;
            maxY = j*voxelSide - areaOfInterest.width/2;                
                columnNbOfPoints(globalIndex) = length(find(...
                    pointCloud(:,1) >= minX & ...
                    pointCloud(:,1) < maxX & ...
                    pointCloud(:,2) >= minY & ...
                    pointCloud(:,2) < maxY));
                
                globalIndex = globalIndex + 1;
        end
    end
    
    % Create histogram
    sampleHist = zeros(1,nbOfBins);
    for i = 1:nbOfBins-1
        sampleHist(i) = length(...
            find(columnNbOfPoints >= ranges(i) &...
            columnNbOfPoints < ranges(i+1)));
    end
    sampleHist(nbOfBins) = ...
        length(find(columnNbOfPoints >= ranges(nbOfBins)));
    
    dataset(sampleIndex).features('histZ') = sampleHist;
end
