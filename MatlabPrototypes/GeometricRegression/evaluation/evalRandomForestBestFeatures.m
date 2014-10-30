disp('Evaluating random forest with the best features...');

evaluation = evaluationStruct;
evaluation.name = 'random forest with best features';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);

useValueThreshold = false; % else choose a number of features
if useValueThreshold
    goodEnoughThreshold = 0.4;
    bestFeaturesIndexes = find(...
        regressionInfo.featuresImportance >= goodEnoughThreshold);
else
    maxNbOfFeaturesThreshold = min(12, ...
        length(regressionInfo.featuresImportance));
    [v bestFeaturesIndexes] = sort(regressionInfo.featuresImportance);
    bestFeaturesIndexes = bestFeaturesIndexes(1:maxNbOfFeaturesThreshold);
end

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(...
        regressionInfo.nbOfTrees,...
        regressionInfo.trainingFeatures(otherIndexes,bestFeaturesIndexes),...
        regressionInfo.trainingLabels(otherIndexes,:),...
        'Method', 'R',...
        'MinLeaf', regressionInfo.nbOfLeaves);
    
    leaveOneOutPrediction(i) = regressor.predict(...
        regressionInfo.trainingFeatures(i,bestFeaturesIndexes));
end

evaluation.labels = leaveOneOutPrediction;
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluation.rSquared = rSquared(regressionInfo.trainingLabels, ...
    evaluation.labels);

evaluations = [evaluations ; evaluation];