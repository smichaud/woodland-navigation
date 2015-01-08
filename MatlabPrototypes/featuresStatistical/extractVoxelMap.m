% Split the area of interest into voxel and create an histogram of nbOfPts
greatestCommonDivisorCM = ...
    gcd(areaOfInterest.depth*100,...
    gcd(areaOfInterest.height*100,...
    areaOfInterest.height*100));
voxelSide = greatestCommonDivisorCM/100;
nbOfX = round(areaOfInterest.depth/voxelSide);
nbOfY = round(areaOfInterest.width/voxelSide);
nbOfZ = round(areaOfInterest.height/voxelSide);
nbOfVoxels = nbOfX*nbOfY*nbOfZ;

nbOfBins = 5; % 5 with max = 40
maxPtsPerVoxel = 10; % Choosed approximately...
ranges = 0:maxPtsPerVoxel/nbOfBins:maxPtsPerVoxel-maxPtsPerVoxel/nbOfBins;

nbOfSamples = length(dataset);
tempCubeInfo = cell(nbOfSamples);
for sampleIndex=1:nbOfSamples
    voxelNbOfPoints = zeros(1,floor(nbOfVoxels));
    pointCloud = dataset(sampleIndex).areaOfInterest;
    groundHeight = dataset(sampleIndex).groundHeight;
    
    tempCubeInfo{sampleIndex} = zeros(nbOfVoxels, 3); % yes I am lazy
    
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
            for k = 1:nbOfZ
                minZ = groundHeight + (k-1)*voxelSide;
                maxZ = groundHeight + k*voxelSide;
                
                voxelNbOfPoints(globalIndex) = length(find(...
                    pointCloud(:,1) >= minX & ...
                    pointCloud(:,1) < maxX & ...
                    pointCloud(:,2) >= minY & ...
                    pointCloud(:,2) < maxY & ...
                    pointCloud(:,3) >= minZ & ...
                    pointCloud(:,3) < maxZ));
                
                tempCubeInfo{sampleIndex}(globalIndex,:) = ...
                    [minX+voxelSide/2, minY+voxelSide/2, minZ+voxelSide];
                globalIndex = globalIndex + 1;
            end
        end
    end
    
    dataset(sampleIndex).features('voxelMap') = voxelNbOfPoints;
end
