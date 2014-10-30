disp('Extracting traversability cost...');

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    switch(traversabilityCostInfo.traversabilityMetrics)
        case traversabilityCostInfo.motorCurrentsIntegralMetric
            [traversabilityCost, startTime, stopTime] = ...
                currentsIntegralTraversability(dataset(i), areaOfInterest, ...
                traversabilityCostInfo.wantToCorrectSlope, robotSpeed);
            
            dataset(i).traversabilityStartTime = startTime;
            dataset(i).traversabilityStopTime = stopTime;
            dataset(i).traversabilityCost = traversabilityCost;
            
        case traversabilityCostInfo.motorCurrentsVarianceMetric
            [traversabilityCost, startTime, stopTime] = ...
                currentsVarianceTraversability(dataset(i),...
                areaOfInterest, robotSpeed);
            
            dataset(i).traversabilityStartTime = startTime;
            dataset(i).traversabilityStopTime = stopTime;
            dataset(i).traversabilityCost = traversabilityCost;
            
        case traversabilityCostInfo.inertiaVarianceMetric
            [traversabilityCost, startTime, stopTime] = ...
                inertiaVarianceTraversability(dataset(i), areaOfInterest,...
                robotSpeed);
            
            dataset(i).traversabilityStartTime = startTime;
            dataset(i).traversabilityStopTime = stopTime;
            dataset(i).traversabilityCost = traversabilityCost;
        case traversabilityCostInfo.randomValueMetric
            dataset(i).traversabilityStartTime = 33;
            dataset(i).traversabilityStopTime = 39;
            dataset(i).traversabilityCost = randi([1 100], 1, 1);
    end
end


