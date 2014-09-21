evaluation = evaluationStruct;
evaluation.name = 'random forest with all features';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPrediction = zeros(nbOfTrainingSamples, 1);

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(...
        regressionInfo.nbOfTrees,...
        regressionInfo.trainingFeatures(otherIndexes,:),...
        regressionInfo.trainingLabels(otherIndexes,:),...
        'Method', 'R',...
        'MinLeaf', regressionInfo.nbOfLeaves);
    
    leaveOneOutPrediction(i) =...
        regressor.predict(regressionInfo.trainingFeatures(i,:));
end

evaluation.labels = leaveOneOutPrediction;
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluations = [evaluations ; evaluation];