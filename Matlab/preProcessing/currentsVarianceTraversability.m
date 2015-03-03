function [traversabilityCost, startTime, stopTime] = ...
    currentsVarianceTraversability(sample, areaOfInterest, robotSpeed)

startPeakCurrent = 4.0; % both should be over that value

noObstacleDuration = areaOfInterest.distFromRobot/robotSpeed;
traversabilityCostDuration = areaOfInterest.depth/robotSpeed;

currents = sample.rawCurrents;
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

startTime = sample.rawCurrents(startIndex, 1);
stopTime = startTime + traversabilityCostDuration;

traversabilityCost = var(currents(startIndex:stopIndex, 2)) + ...
    var(currents(startIndex:stopIndex, 3));

end

