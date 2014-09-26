% Create a sample for every 'stepSize' interval
samples = [];
for i = 1:length(rawData)
    startTime = 2;
    endTime = min(motorCurrents(end,1), rawData(1).rollPitchYaw(end,1)) - 2;
    stepSize = 0.5;
    
    for j = startTime:stepSize:endTime-stepSize
        sample = mean()
    end
end


% interpolatedSamples = [2::];


