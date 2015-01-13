function histogram = getRatioGridsHistogram(...
    data, clusterCenters)

nbOfCenters = size(clusterCenters, 1);
nbOfRatioGrids = size(data.ratioGridVectors, 1);
histogram = zeros(1,nbOfCenters);

% 
nearestCenters = dsearchn(clusterCenters, data.ratioGridVectors);
histogram = histc(nearestCenters, 1:nbOfCenters);

end

