disp('Extracting traversability cost...');

standbyCurrent = 0.5900; % normally
startPeakCurrent = 4.0; % both should be over that value

noObstacleDuration = areaOfInterest.distFromRobot/robotSpeed;
traversabilityCostDuration = areaOfInterest.depth/robotSpeed;

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    currents = dataset(i).rawCurrents;
    acquisitionTimeSteps = currents(2,1)-currents(1,1);
    
    startIndex = -1;
    for y=1:size(currents,1)
        if currents(y,2) > startPeakCurrent && currents(y,3) > startPeakCurrent
           startIndex = y + round(noObstacleDuration/acquisitionTimeSteps);
           break;
        end
    end
    
    if startIndex == -1
        error('No robot move start found in motor currents')
    end
    
    stopIndex = startIndex + ...
        round(traversabilityCostDuration/acquisitionTimeSteps);
    
    dataset(i).traversabilityStartIndex = startIndex;
    dataset(i).traversabilityStopIndex = stopIndex;
    
    traversabilityCost = 0;
    for j = startIndex:stopIndex-1
      current1 = (dataset(i).rawCurrents(j,2) + dataset(i).rawCurrents(j+1,2))/2;
      current2 = (dataset(i).rawCurrents(j,3) + dataset(i).rawCurrents(j+1,3))/2;
      traversabilityCost = traversabilityCost + ...
          acquisitionTimeSteps*(current1+current2);
    end
    
    dataset(i).traversabilityCost = traversabilityCost;    
end

