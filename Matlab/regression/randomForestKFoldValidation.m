function [rSquaredMean] = randomForestKFoldValidation(...
    nbOfTrees, nbOfLeaves, features, labels, nbOfFold)

nbOfTrainingSamples = size(features,1);
sizesVector = ones(1,nbOfFold)*floor(nbOfTrainingSamples/nbOfFold);
sizesVector(1:mod(nbOfTrainingSamples,nbOfFold)) = ...
    sizesVector(1:mod(nbOfTrainingSamples,nbOfFold)) + 1;

leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);
stopIndex = 0;
for inc = sizesVector
    startIndex = stopIndex + 1;
    stopIndex = startIndex + inc - 1;
    
    testIndexes = startIndex:stopIndex;
    trainIndexes = 1:nbOfTrainingSamples;
    trainIndexes(testIndexes) = [];
    
    regressor = TreeBagger(nbOfTrees,...
        features(trainIndexes,:), labels(trainIndexes,:),...
        'Method', 'R', 'MinLeaf', nbOfLeaves);
    
    leaveOneOutPrediction(testIndexes) = ...
        regressor.predict(features(testIndexes,:));
end

rSquaredMean = rSquared(labels, leaveOneOutPrediction);

end

