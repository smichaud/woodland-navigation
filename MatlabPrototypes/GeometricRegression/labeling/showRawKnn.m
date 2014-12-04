nbOfSamples = length(dataset);

usedIMU = [2,3,4,5,6,7];

startTime = dataset(sampleIndex).traversabilityStartTime;
endTime = dataset(sampleIndex).traversabilityStopTime;
samplingTime = 1/20; % 1/frequency
signalLength = floor((endTime-startTime)/samplingTime);
timeVector = (0:signalLength-1)*samplingTime;

rawVectors = zeros(nbOfSamples, signalLength, length(usedIMU));
for sampleIndex = 1:nbOfSamples
    for imuIndex = 1:length(usedIMU)
        rawVectors(sampleIndex, :, imuIndex) = interp1(...
            dataset(sampleIndex).rawIMU(:,1),...
            dataset(sampleIndex).rawIMU(:,usedIMU(imuIndex)),...
            startTime:samplingTime:(startTime ...
            +(length(timeVector)-1)*samplingTime));
    end
end
rawVectors = reshape(...
    rawVectors, nbOfSamples, signalLength*length(usedIMU));

minkowskiDistance = 1;
distanceMatrix = squareform(...
    pdist(rawVectors, 'minkowski', minkowskiDistance));

for i = 1:nbOfSamples
    [vector orderedIndexes] = sort(distanceMatrix(i,:));
    
    figure('Name', sprintf('Nearest Neighbor (IMU) %d of %d',i,nbOfSamples),...
        'units','normalized', 'outerposition',[0 0 1 1]);
    subplot(2,2,1);
    imshow(dataset(i).image);
    title('Example')
    subplot(2,2,2);
    imshow(dataset(orderedIndexes(2)).image);
    title(sprintf('1st nearest (%f)', vector(2)));
    subplot(2,2,3);
    imshow(dataset(orderedIndexes(3)).image);
    title(sprintf('2nd nearest (%f)', vector(3)));
    subplot(2,2,4);
    imshow(dataset(orderedIndexes(4)).image);
    title(sprintf('3rd nearest (%f)', vector(4)));
    
    waitforbuttonpress;
    close all;
end