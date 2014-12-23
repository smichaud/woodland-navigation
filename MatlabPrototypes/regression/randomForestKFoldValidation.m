function [rSquaredMean] = randomForestKFoldValidation(...
    nbOfTrees, nbOfLeaves, features, labels, nbOfFold)

nbOfTrainingSamples = length(features);
foldSize = round(nbOfTrainingSamples/nbOfFold);

leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);
for i = 0:nbOfFold-1
    startIndex = i*foldSize + 1;
    stopIndex = startIndex - 1 + foldSize;
    if stopIndex > nbOfTrainingSamples
        stopIndex = nbOfTrainingSamples;
    end
    testIndexes = startIndex:stopIndex;
    trainIndexes = 1:nbOfTrainingSamples;
    trainIndexes(testIndexes) = [];
    
    features(trainIndexes,:)
    labels(trainIndexes,:)
    
    regressor = TreeBagger(nbOfTrees,...
        features(trainIndexes,:), labels(trainIndexes,:),...
        'Method', 'R', 'MinLeaf', nbOfLeaves);
    
    leaveOneOutPrediction(testIndexes) = ...
        regressor.predict(features(testIndexes,:));
end

rSquaredResult = rSquared(labels, leaveOneOutPrediction);

end
