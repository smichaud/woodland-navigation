function [traversabilityCost, startTime, stopTime] = ...
    inertiaVarianceTraversability(sample, areaOfInterest, robotSpeed)
% I interpolate because time step does not seem to be constant

startPeakXInertia = 0.075;

noObstacleDuration = areaOfInterest.distFromRobot/robotSpeed;
traversabilityCostDuration = areaOfInterest.depth/robotSpeed;

inertialMeasurements = sample.rawInertia;

startTime = -1;
for i=1:size(inertialMeasurements,1) - 1
    if inertialMeasurements(i+1,2) - inertialMeasurements(i,2) > startPeakXInertia
        startTime = inertialMeasurements(i,1) + noObstacleDuration;
        stopTime = startTime + traversabilityCostDuration;
        break;
    end
end

if startTime == -1
    error('No robot move start found in motor currents')
end

stepSize = 0.001;
timeVector = (startTime:stepSize:stopTime)';
inertiaInArea = interp1(...
    inertialMeasurements(:,1),...
    inertialMeasurements(:,2),...
    timeVector);
inertiaInArea = [timeVector inertiaInArea];

traversabilityCost = var(inertiaInArea(:, 2));
end