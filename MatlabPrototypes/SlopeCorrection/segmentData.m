% Create a sample for every timeInterval (using mean as low pass filter)
for i = 1:length(rawData)
    startTime = 2;
    endTime = min(rawData(i).motorCurrents(end,1), ...
        rawData(i).rollPitchYaw(end,1)) - 2;
    stepSize = 0.001;
    
    timeVector = startTime:stepSize:endTime;
    motorCurrents1 = interp1(...
        rawData(i).motorCurrents(:,1),...
        rawData(i).motorCurrents(:,2),...
        timeVector);
    motorCurrents2 = interp1(...
        rawData(i).motorCurrents(:,1),...
        rawData(i).motorCurrents(:,3),...
        timeVector);
    motorCurrents = motorCurrents1 + motorCurrents2;
    pitch = interp1(...
        rawData(i).rollPitchYaw(:,1),...
        rawData(i).rollPitchYaw(:,3),...
        timeVector);    
    
    timeInterval = 0.1;
    indexesPerSample = timeInterval/stepSize;
    for j = 0:indexesPerSample:length(timeVector)-indexesPerSample
        samplePitch = mean(pitch(round(j)+1:round(j+indexesPerSample)));
        sampleMotorCurrents = mean(...
            motorCurrents(round(j)+1:round(j+indexesPerSample)));
        
        pitchMotorCurrentsSamples = [pitchMotorCurrentsSamples ; 
            samplePitch, sampleMotorCurrents];
    end
end

