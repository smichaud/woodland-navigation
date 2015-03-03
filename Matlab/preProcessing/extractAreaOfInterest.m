disp('Extracting area of interest (from point cloud)...');

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    pointCloud = dataset(i).rawPointCloud(:,1:3);
    
    minX = areaOfInterest.distFromRobot + areaOfInterest.xTfAdjustment;
    maxX = areaOfInterest.distFromRobot + areaOfInterest.xTfAdjustment +...
        areaOfInterest.depth;
    minY = - areaOfInterest.width/2;
    maxY = areaOfInterest.width/2;    
    pointCloud = pointCloud(find(...
                    pointCloud(:,1) >= minX & ...
                    pointCloud(:,1) < maxX & ...
                    pointCloud(:,2) >= minY & ...
                    pointCloud(:,2) < maxY),:);
    
    dataset(i).groundHeight = min(pointCloud(:,3));
    minZ = dataset(i).groundHeight + areaOfInterest.groundThreshold;
    maxZ = dataset(i).groundHeight + areaOfInterest.height;    
    pointCloud = pointCloud(find(...
                    pointCloud(:,3) >= minZ & ...
                    pointCloud(:,3) < maxZ),:);
    
    dataset(i).areaOfInterest = pointCloud;
end