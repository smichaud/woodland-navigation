function dataset = buildPointsCountVoxelGrids(...
    dataset, voxelSize, areaOfInterest)
areaMinX = areaOfInterest(1,1);
areaMaxX = areaOfInterest(1,2);
areaMinY = areaOfInterest(2,1);
areaMaxY = areaOfInterest(2,2);
areaMinZ = areaOfInterest(3,1);
areaMaxZ = areaOfInterest(3,2);

nbOfX = (areaMaxX - areaMinX)/voxelSize;
nbOfY = (areaMaxY - areaMinY)/voxelSize;
nbOfZ = (areaMaxZ - areaMinZ)/voxelSize;
if rem(nbOfX,1) || rem(nbOfX,1) || rem(nbOfX,1)
    warning('areaOfInterest divided by voxelSize are are not integers');
end
nbOfX = round(nbOfX);
nbOfY = round(nbOfY);
nbOfZ = round(nbOfZ);

samplesCount = length(dataset);
for sampleIndex = 1:samplesCount
    voxelsCardinalityMap = zeros(nbOfX, nbOfY, nbOfZ);
    for i = 1:nbOfX
        minX = areaMinX + (i-1)*voxelSize;
        maxX = areaMinX + i*voxelSize;
        for j = 1:nbOfY
            minY = areaMinY + (j-1)*voxelSize;
            maxY = areaMinY + j*voxelSize;
            for k = 1:nbOfZ
                minZ = areaMinZ + (k-1)*voxelSize;
                maxZ = areaMinZ + k*voxelSize;

                voxelsCardinalityMap(i,j,k) = length(find(...
                    dataset(sampleIndex).pointcloud(:,1) >= minX & ...
                    dataset(sampleIndex).pointcloud(:,1) < maxX & ...
                    dataset(sampleIndex).pointcloud(:,2) >= minY & ...
                    dataset(sampleIndex).pointcloud(:,2) < maxY & ...
                    dataset(sampleIndex).pointcloud(:,3) >= minZ & ...
                    dataset(sampleIndex).pointcloud(:,3) < maxZ));

                dataset(sampleIndex).voxelsCardinalityMap = ...
                    voxelsCardinalityMap;
            end
        end
    end
end

end

