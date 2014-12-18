nbOfSamples = length(dataset);

for sampleIndex = 1:nbOfSamples
    dataset(sampleIndex).dftIMU = [];
    
    startTime = dataset(sampleIndex).traversabilityStartTime;
    endTime = dataset(sampleIndex).traversabilityStopTime;
    samplingFrequency = 20; % Hz
    samplingTime = 1/samplingFrequency; % sec
    signalLength = floor((endTime-startTime)/samplingTime);
    timeVector = (0:signalLength-1)*samplingTime;
    
    for imuIndex = 2:7
        dataVector = interp1(...
            dataset(sampleIndex).rawIMU(:,1),...
            dataset(sampleIndex).rawIMU(:,imuIndex),...
            startTime:samplingTime:(startTime ...
            +(length(timeVector)-1)*samplingTime));
        
        NFFT = 2^nextpow2(signalLength);
        Y = fft(dataVector,NFFT)/signalLength;
        f = samplingFrequency/2*linspace(0,1,NFFT/2+1);
        
        if isempty(dataset(sampleIndex).dftIMU)
            dataset(sampleIndex).dftIMU = zeros(length(f), 7);
            dataset(sampleIndex).dftIMU(:,1) = f';
        end
        dataset(sampleIndex).dftIMU(:,imuIndex) = (2*abs(Y(1:NFFT/2+1)))';
    end
end

