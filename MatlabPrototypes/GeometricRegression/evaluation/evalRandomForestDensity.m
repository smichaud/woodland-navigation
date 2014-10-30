disp('Evaluating random forest with density only...');

evaluation = evaluationStruct;
evaluation.name = 'random forest with density';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);

densityIndex = find(ismember(regressionInfo.featureNames,'density'));

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(...
        regressionInfo.nbOfTrees,...
        regressionInfo.trainingFeatures(otherIndexes,densityIndex),...
        regressionInfo.trainingLabels(otherIndexes,:),...
        'Method', 'R',...
        'MinLeaf', regressionInfo.nbOfLeaves);
    
    leaveOneOutPrediction(i) =...
        regressor.predict(regressionInfo.trainingFeatures(i,densityIndex));
end

evaluation.labels = leaveOneOutPrediction;
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluation.rSquared = rSquared(regressionInfo.trainingLabels, ...
    evaluation.labels);

evaluations = [evaluations ; evaluation];