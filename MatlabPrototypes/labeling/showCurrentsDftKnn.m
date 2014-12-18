nbOfSamples = length(dataset)

removeIndexes = [];
for sampleIndex = 1:nbOfSamples
    if ~isempty(strfind(dataset(sampleIndex).name, '2014-10-13'))
        removeIndexes = [removeIndexes sampleIndex];
    end
end
dataset(removeIndexes) = [];

nbOfSamples = length(dataset)

currentsDftLength = size(dataset(1).dftCurrents, 1);
currentsDftVectors = zeros(nbOfSamples, 2*currentsDftLength);
for sampleIndex = 1:nbOfSamples
    currentsDftVectors(sampleIndex,:) = ...
        [dataset(sampleIndex).dftCurrents(:,2)' ...
        dataset(sampleIndex).dftCurrents(:,3)'];
end

displayConfiguration = [4 2];
nbOfSamplePerFigure = prod(displayConfiguration);

vectorSize = size(currentsDftVectors,2);

featureNorms = zeros(nbOfSamples,1);
for i = 1:nbOfSamples
%     featureNorms(i) = norm(currentsDftVectors(i,:));
    featureNorms(i) = sum(currentsDftVectors(i,:));
end

distanceMatrix = squareform(...
    pdist(currentsDftVectors, 'minkowski', minkowskiDistance));

[sortedValues,sortedIndexes] = sort(featureNorms);

for i = 1:nbOfSamples
    [vector orderedIndexes] = sort(distanceMatrix(sortedIndexes(i),:));
    
    figure('Name', sprintf(...
        'Nearest Neighbor (IMU) %d of %d', i, nbOfSamples),...
        'units','normalized','outerposition',[0 0 1 1]);
    subplot(2,2,1);
    imshow(dataset(sortedIndexes(i)).image);
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
