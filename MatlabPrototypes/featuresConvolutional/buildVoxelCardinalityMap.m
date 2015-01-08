if mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide) ||...
        mod(areaOfInterest.depth,voxelSide)
    warning('Voxel side size does not fit into the area of interest');
end

nbOfX = round(areaOfInterest.depth/voxelSide);
nbOfY = round(areaOfInterest.width/voxelSide);
nbOfZ = round(areaOfInterest.height/voxelSide);

nbOfSamples = length(dataset);
for sampleIndex = 1:nbOfSamples;
    pointCloud = dataset(sampleIndex).areaOfInterest;
    groundHeight = dataset(sampleIndex).groundHeight;
    
    voxelMap = zeros(nbOfX, nbOfY, nbOfZ);
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
                
                voxelMap(i,j,k) = length(find(...
                    pointCloud(:,1) >= minX & ...
                    pointCloud(:,1) < maxX & ...
                    pointCloud(:,2) >= minY & ...
                    pointCloud(:,2) < maxY & ...
                    pointCloud(:,3) >= minZ & ...
                    pointCloud(:,3) < maxZ));
            end
        end
    end
    dataset(sampleIndex).voxelMap = voxelMap;
end
