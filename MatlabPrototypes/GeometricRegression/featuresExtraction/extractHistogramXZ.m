% ranges example : [0,50,...,MAX] => [0,50[, [50,100[, ... MAX[

nbPointsMax = 1000; % for now, the max in dataset is 845
nbOfBin = 4;
nbOfLayers = 10;
maxForLayers = nbPointsMax/nbOfLayers;
ranges = 0:maxForLayers/nbOfBin:maxForLayers-maxForLayers/nbOfBin;
layerHeight = areaOfInterest.height/nbOfLayers;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    layersXY = zeros(1,nbOfLayers);
    pointCloud = dataset(i).areaOfInterest;
    groundHeight = dataset(i).groundHeight;
    
    % Create the layers and save nb of pts
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
    
    % Create histogram
    sampleHist = zeros(1,nbOfBin);
    for j = 1:nbOfBin-1
        sampleHist(j) = length(...
            find(layersXY >= ranges(j) & layersXY < ranges(j+1)));
    end
    sampleHist(nbOfBin) = length(find(layersXY >= ranges(nbOfBin)));
    
    dataset(i).features('histXY') = sampleHist;
end
