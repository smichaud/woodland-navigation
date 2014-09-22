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

nbOfBins = 6;
maxPtsPerVoxel = 50; % Find a good value here
ranges = 0:maxPtsPerVoxel/nbOfBins:maxPtsPerVoxel-maxPtsPerVoxel/nbOfBins;

nbOfSamples = length(regressionInfo.trainingLabels);
for sampleIndex=1:nbOfSamples
    voxelNbOfPoints = zeros(1,floor(nbOfVoxels));
    pointCloud = dataset(sampleIndex).areaOfInterest;
    groundHeight = dataset(sampleIndex).groundHeight;
    
    % Create the voxel and save nb of pts
    globalIndex = 1;
    for i = 1:nbOfX
        xPointCloud = pointCloud;
        minX = (i-1)*voxelSide + areaOfInterest.distFromRobot;
        maxX = i*voxelSide + areaOfInterest.distFromRobot;
        xPointCloud = xPointCloud(find(xPointCloud(:,1) > ...
            repmat(minX, size(xPointCloud,1),1)),:);
        xPointCloud = xPointCloud(find(xPointCloud(:,1) <= ...
            repmat(maxX, size(xPointCloud,1),1)),:);
        
        for j = 1:nbOfY
            yPointCloud = xPointCloud;
            minY = (j-1)*voxelSide - areaOfInterest.width/2;
            maxY = j*voxelSide + areaOfInterest.width/2;
            yPointCloud = yPointCloud(find(yPointCloud(:,2) > ...
                repmat(minY, size(yPointCloud,1),1)),:);
            yPointCloud = yPointCloud(find(yPointCloud(:,2) <= ...
                repmat(maxY, size(yPointCloud,1),1)),:);
            
            for k = 1:nbOfZ
                voxelPointCloud = yPointCloud;
                minZ = groundHeight + (k-1)*voxelSide;
                maxZ = groundHeight + k*voxelSide;
                voxelPointCloud = voxelPointCloud(...
                    find(voxelPointCloud(:,3) > ...
                    repmat(minZ, size(voxelPointCloud,1),1)),:);
                voxelPointCloud = voxelPointCloud(find(...
                    voxelPointCloud(:,3) <= ...
                    repmat(maxZ, size(voxelPointCloud,1),1)), : );

                voxelNbOfPoints(i) = size(voxelPointCloud,1);
                globalIndex = globalIndex + 1;                
            end
        end
    end
    
    disp(['Total nb pts:' num2str(length(pointCloud)) ' ?= ' num2str(sum(voxelNbOfPoints))]);
    
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
