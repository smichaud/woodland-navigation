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
    
    % Compute the corrected traversability cost
    stepSize = 0.001;
    startTime = dataset(i).rawCurrents(startIndex, 1);
    endTime = startTime + traversabilityCostDuration;
    timeVector = (startTime:stepSize:endTime)';
    
    motorCurrents1 = interp1(...
        dataset(i).rawCurrents(:,1),...
        dataset(i).rawCurrents(:,2),...
        timeVector);
    motorCurrents2 = interp1(...
        dataset(i).rawCurrents(:,1),...
        dataset(i).rawCurrents(:,3),...
        timeVector);
    motorCurrents = motorCurrents1 + motorCurrents2;
    pitch = interp1(...
        dataset(i).rollPitchYaw(:,1),...
        dataset(i).rollPitchYaw(:,3),...
        timeVector);
    
    dataset(i).traversabilityCost = sum(motorCurrents)*stepSize;
    
%     predictedCurrents = pitch(:,1)*slopeCorrection(2) + ...
%         slopeCorrection(1);
%     
%     dataset(i).traversabilityCost = ...
%         sum(motorCurrents-predictedCurrents)*stepSize;
end

