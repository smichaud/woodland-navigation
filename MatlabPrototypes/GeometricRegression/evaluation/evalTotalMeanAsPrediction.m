evaluation = evaluationStruct;
evaluation.name = 'total mean as label';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPrediction = repmat(nbOfTrainingSamples, 1);

evaluation.labels = repmat(mean(regressionInfo.trainingLabels), ...
    nbOfTrainingSamples, 1);

evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluation.rSquared = rSquared(regressionInfo.trainingLabels, ...
    evaluation.labels);

evaluations = [evaluations ; evaluation];