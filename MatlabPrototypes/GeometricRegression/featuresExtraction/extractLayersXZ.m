% Split the point cloud into layers parallel to the XZ plane.
% The number of point for each layer is a feature
nbOfLayers = 8;
layerWidth = areaOfInterest.width/nbOfLayers;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    layersXZ = zeros(1,nbOfLayers);
    pointCloud = dataset(i).areaOfInterest;
    
    for j = 1:nbOfLayers
        minY = (j-1)*layerWidth - areaOfInterest.width/2;
        maxY = j*layerWidth - areaOfInterest.width/2;
        
        layersXZ(j) = length(find(...
            pointCloud(:,2) >= minY & ...
            pointCloud(:,2) < maxY));
    end
    
    dataset(i).features('layersXZ') = layersXZ;
end