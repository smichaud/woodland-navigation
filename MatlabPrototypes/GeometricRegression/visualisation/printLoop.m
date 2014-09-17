nbOfSamples = length(dataset);
for i=1:nbOfSamples
    nbOfPoints(i) = size(dataset(i).areaOfInterest,1);
%     disp(sprintf('%s : %f === %f',...
%         dataset(i).name, ...
%         dataset(i).features('density'), ...
%         dataset(i).features('highestPoint')));
end

minNbOfPoints = min(nbOfPoints)
maxNbOfPoints = max(nbOfPoints)
meanNbOfPoints = mean(nbOfPoints)
medianNbOfPoints = median(nbOfPoints)

