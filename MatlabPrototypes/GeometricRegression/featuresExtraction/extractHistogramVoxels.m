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
maxPtsPerVoxel = 40; % Choose approximately...
ranges = 0:maxPtsPerVoxel/nbOfBins:maxPtsPerVoxel-maxPtsPerVoxel/nbOfBins;

test = [];

nbOfSamples = length(regressionInfo.trainingLabels);
for sampleIndex=1:nbOfSamples
    voxelNbOfPoints = zeros(1,floor(nbOfVoxels));
    pointCloud = dataset(sampleIndex).areaOfInterest;
    groundHeight = dataset(sampleIndex).groundHeight;
    
    % Create the voxel and save nb of pts
    globalIndex = 1;
    for i = 1:nbOfX
        minX = (i-1)*voxelSide + areaOfInterest.distFromRobot;
        maxX = i*voxelSide + areaOfInterest.distFromRobot;
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
                
                globalIndex = globalIndex + 1;
            end
        end
    end
    
    % Create histogram
    sampleHist = zeros(1,nbOfBins);
    for i = 1:nbOfBins-1
        sampleHist(i) = length(...
            find(voxelNbOfPoints >= ranges(i) &...
            voxelNbOfPoints < ranges(i+1)));
    end
    sampleHist(nbOfBins) = ...
        length(find(voxelNbOfPoints >= ranges(nbOfBins)));
    
    dataset(sampleIndex).features('voxelsHist') = sampleHist;
end
