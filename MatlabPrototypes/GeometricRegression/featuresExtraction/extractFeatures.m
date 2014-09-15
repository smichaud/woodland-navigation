volume = areaOfInterest.width * areaOfInterest.height ...
    * areaOfInterest.depth;

for i=1:nbOfSamples
    dataset(i).features = containers.Map; % Prevent dataset from sharing the features
    
    nbOfPoints = size(dataset(i).areaOfInterest,1);
    dataset(i).features('density') = nbOfPoints/volume;
    
    if nbOfPoints == 0
        dataset(i).features('highestPoint') = 0;
    else
        pointHeight = max(dataset(i).areaOfInterest(:,3));
        dataset(i).features('highestPoint') = pointHeight - ...
            dataset(i).groundHeight + areaOfInterest.groundThreshold;
    end
    
    % Others features to come :) 
end
