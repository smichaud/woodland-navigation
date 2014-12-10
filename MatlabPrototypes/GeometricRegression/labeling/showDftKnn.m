initLabeling;

distanceMatrix = squareform(...
    pdist(dftVectors, 'minkowski', minkowskiDistance));

featureNorms = zeros(nbOfSamples,1);
for i = 1:nbOfSamples
%     featureNorms(i) = norm(dftVectors(i,:));
    featureNorms(i) = sum(dftVectors(i,:));
end

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