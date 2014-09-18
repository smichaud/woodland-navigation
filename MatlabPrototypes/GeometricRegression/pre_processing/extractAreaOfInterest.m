disp('Extracting area of interest (from point cloud)...');

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    pointCloud = dataset(i).rawPointCloud(:,1:3);
    
    % Crop x (robot front)
    pointCloud = pointCloud(find(pointCloud(:,1) >= ...
        repmat(areaOfInterest.distFromRobot, size(pointCloud,1),1)), : );
    pointCloud = pointCloud(find(pointCloud(:,1) <= ...
        repmat(areaOfInterest.distFromRobot + areaOfInterest.depth,...
        size(pointCloud,1),1)), : );
    
    % Crop y (robot side)
    pointCloud = pointCloud(find(pointCloud(:,2) >= ...
        repmat(-areaOfInterest.width/2,size(pointCloud,1),1)), : );
    pointCloud = pointCloud(find(pointCloud(:,2) <= ...
        repmat(areaOfInterest.width/2,size(pointCloud,1),1)), : );
    
    % Crop z (height)
    dataset(i).groundHeight = min(pointCloud(:,3));
    pointCloud = pointCloud(find(pointCloud(:,3) >= ...
        repmat(dataset(i).groundHeight + areaOfInterest.groundThreshold,...
        size(pointCloud,1),1)), : );
    pointCloud = pointCloud(find(pointCloud(:,3) <= ...
        repmat(dataset(i).groundHeight+areaOfInterest.height, ...
        size(pointCloud,1),1)), : );
    
    dataset(i).areaOfInterest = pointCloud;
end