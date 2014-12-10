function [rSquaredResult] = randomForestleaveOneOut(...
    nbOfTrees, nbOfLeaves, features, labels)

nbOfTrainingSamples = length(features);
leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(nbOfTrees,...
        features(otherIndexes,:), labels(otherIndexes,:),...
        'Method', 'R', 'MinLeaf', nbOfLeaves);
    
    leaveOneOutPrediction(i) = ...
        regressor.predict(features(i,:));
end

rSquaredResult = rSquared(labels, leaveOneOutPrediction);

end

