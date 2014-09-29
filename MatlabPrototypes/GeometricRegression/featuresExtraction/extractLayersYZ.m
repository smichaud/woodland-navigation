% Split the point cloud into layers parallel to the YZ plane.
% The number of point for each layer is a feature
nbOfLayers = 6;
layerDepth = areaOfInterest.depth/nbOfLayers;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    layersYZ = zeros(1,nbOfLayers);
    pointCloud = dataset(i).areaOfInterest;
    
    for j = 1:nbOfLayers
        minX = (j-1)*layerDepth + areaOfInterest.distFromRobot + ...
            areaOfInterest.xTfAdjustment;
        maxX = j*layerDepth + areaOfInterest.distFromRobot + ...
            areaOfInterest.xTfAdjustment;
        
        layersYZ(j) = length(find(...
            pointCloud(:,1) >= minX & ...
            pointCloud(:,1) < maxX));
    end
    
    dataset(i).features('layersYZ') = layersYZ;
end