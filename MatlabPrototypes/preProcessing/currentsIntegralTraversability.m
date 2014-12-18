function [traversabilityCost, startTime, stopTime] = ...
    currentsIntegralTraversability(...
    sample,...
    areaOfInterest,...
    wantToCorrectSlope,...
    robotSpeed)
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

% Compute the corrected traversability cost
stepSize = 0.001;
startTime = sample.rawCurrents(startIndex, 1);
stopTime = startTime + traversabilityCostDuration;
timeVector = (startTime:stepSize:stopTime)';

motorCurrents1 = interp1(...
    sample.rawCurrents(:,1),...
    sample.rawCurrents(:,2),...
    timeVector);
motorCurrents2 = interp1(...
    sample.rawCurrents(:,1),...
    sample.rawCurrents(:,3),...
    timeVector);
motorCurrents = motorCurrents1 + motorCurrents2;
pitch = interp1(...
    sample.rollPitchYaw(:,1),...
    sample.rollPitchYaw(:,3),...
    timeVector);

if wantToCorrectSlope == false
    traversabilityCost = sum(motorCurrents)*stepSize;
else
    predictedCurrents = pitch(:,1)*slopeCorrection(2) + ...
        slopeCorrection(1);
    
    traversabilityCost = ...
        sum(motorCurrents-predictedCurrents)*stepSize;
end

end

