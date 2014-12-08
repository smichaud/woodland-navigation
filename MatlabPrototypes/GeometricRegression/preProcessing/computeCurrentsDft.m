nbOfSamples = length(dataset);

for sampleIndex = 1:nbOfSamples
    dataset(sampleIndex).dftCurrents = [];
    
    startTime = dataset(sampleIndex).traversabilityStartTime;
    endTime = dataset(sampleIndex).traversabilityStopTime;
    samplingTime = 1/10; % sec
    signalLength = floor((endTime-startTime)/samplingTime);
    timeVector = (0:signalLength-1)*samplingTime;
    
    for currentsIndex = 2:3
        dataVector = interp1(...
            dataset(sampleIndex).rawCurrents(:,1),...
            dataset(sampleIndex).rawCurrents(:,currentsIndex),...
            startTime:samplingTime:(startTime ...
            +(length(timeVector)-1)*samplingTime));
        
        NFFT = 2^nextpow2(signalLength);
        Y = fft(dataVector,NFFT)/signalLength;
        f = samplingFrequency/2*linspace(0,1,NFFT/2+1);
        
        if isempty(dataset(sampleIndex).dftCurrents)
            dataset(sampleIndex).dftCurrents = zeros(length(f), 3);
            dataset(sampleIndex).dftCurrents(:,1) = f';
        end
        dataset(sampleIndex).dftCurrents(:,currentsIndex) = ...
            (2*abs(Y(1:NFFT/2+1)))';
    end
end
