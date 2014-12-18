function rSquaredResult = knnLeaveOneOut(...
    K, minkowskiDist, features, labels)

nbOfTrainingSamples = length(features);
nbOfLabelFeatures = size(labels,2);
leaveOneOutPredictions = zeros(nbOfTrainingSamples, nbOfLabelFeatures);

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    leaveOneOutPredictions(i,:) = knnRegression(...
        features(otherIndexes,:), labels(otherIndexes,:),...
        features(i,:), K, minkowskiDist);
end

rSquaredResult = rSquared(labels, leaveOneOutPredictions);

end

