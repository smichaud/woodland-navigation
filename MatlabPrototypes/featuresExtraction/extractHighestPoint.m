nbOfSamples = length(dataset);
for i=1:nbOfSamples
    nbOfPoints = size(dataset(i).areaOfInterest,1);
    if nbOfPoints == 0
        dataset(i).features('highestPoint') = 0;
    else
        pointHeight = max(dataset(i).areaOfInterest(:,3));
        dataset(i).features('highestPoint') = pointHeight - ...
            dataset(i).groundHeight + areaOfInterest.groundThreshold;
    end
end
