% Split the point cloud into layers parallel to the XY plane.
% The number of point for each layer is a feature
nbOfLayers = 3;
layerHeight = areaOfInterest.height/nbOfLayers;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    layersXY = zeros(1,nbOfLayers);
    pointCloud = dataset(i).areaOfInterest;
    groundHeight = dataset(i).groundHeight;
    
    for j = 1:nbOfLayers
        layerPointCloud = pointCloud;
        minHeight = groundHeight + (j-1)*layerHeight;
        maxHeight = groundHeight + j*layerHeight;
        layerPointCloud = layerPointCloud(find(layerPointCloud(:,3) > ...
            repmat(minHeight, size(layerPointCloud,1),1)),:);
        layerPointCloud = layerPointCloud(find(layerPointCloud(:,3) <= ...
            repmat(maxHeight, size(layerPointCloud,1),1)), : );
        
        layersXY(j) = size(layerPointCloud,1);
    end
    
    dataset(i).features('layersXY') = layersXY;
end