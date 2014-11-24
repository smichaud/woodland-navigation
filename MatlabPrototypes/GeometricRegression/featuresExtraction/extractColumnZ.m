% Split the area of interest into voxel and create an histogram of nbOfPts
greatestCommonDivisor = ...
    gcd(areaOfInterest.depth*100,areaOfInterest.height*100);
voxelSideX = (greatestCommonDivisor/100)*2;
voxelSideY = greatestCommonDivisor/100;
nbOfX = round(areaOfInterest.depth/voxelSideX);
nbOfY = round(areaOfInterest.width/voxelSideY);
nbOfVoxels = nbOfX*nbOfY;

nbOfSamples = length(dataset);
for sampleIndex=1:nbOfSamples
    columnNbOfPoints = zeros(1,nbOfVoxels);
    pointCloud = dataset(sampleIndex).areaOfInterest;
    
    % Create the voxel and save nb of pts
    globalIndex = 1;
    for i = 1:nbOfX
        minX = (i-1)*voxelSideX + areaOfInterest.distFromRobot +...
            areaOfInterest.xTfAdjustment;
        maxX = i*voxelSideX + areaOfInterest.distFromRobot + ...
            areaOfInterest.xTfAdjustment;
        for j = 1:nbOfY
            minY = (j-1)*voxelSideY - areaOfInterest.width/2;
            maxY = j*voxelSideY - areaOfInterest.width/2;                
                columnNbOfPoints(globalIndex) = length(find(...
                    pointCloud(:,1) >= minX & ...
                    pointCloud(:,1) < maxX & ...
                    pointCloud(:,2) >= minY & ...
                    pointCloud(:,2) < maxY));
                
                globalIndex = globalIndex + 1;
        end
    end
    
    dataset(sampleIndex).features('colZ') = columnNbOfPoints;
end
