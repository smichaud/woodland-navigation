disp('Extracting traversability cost...');

% Approx the time of the traversability zone (constant acquisition script)
defaultStartTime = 30; % sec
defaultStopTime = defaultStartTime + areaOfInterest.depth/robotSpeed;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    dataset(i).traversabilityStartTime = defaultStartTime;
    dataset(i).traversabilityStopTime = defaultStopTime;
    
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
            
        case traversabilityCostInfo.imuDftMinkowski1FromOrigin
            dataset(i).traversabilityCost = ...
                sum(dftVectors(i,:));
            
        case traversabilityCostInfo.imuDftMinkowski2FromOrigin
            dataset(i).traversabilityCost = ...
                norm(dftVectors(i,:));
            
        case traversabilityCostInfo.odometryErrorMetric 
            % Just the 2D translation error for now
            dataset(i).traversabilityCost = ...
                norm(dataset(i).icpOdometry(1:2,4) - [4 ; 0]);
            
        case traversabilityCostInfo.randomValueMetric
            dataset(i).traversabilityCost = randi([1 100], 1, 1);
    end
end


