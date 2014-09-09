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
    groundHeight = min(pointCloud(:,3));
    groundThreshold = 0.12; % Lower than half wheel and robot body (12 cm)
    pointCloud = pointCloud(find(pointCloud(:,3) >= ...
        repmat(groundHeight + groundThreshold ,size(pointCloud,1),1)), : );
    pointCloud = pointCloud(find(pointCloud(:,3) <= ...
        repmat(groundHeight+areaOfInterest.height, ...
        size(pointCloud,1),1)), : );
    
    dataset(i).areaOfInterest = pointCloud;
end