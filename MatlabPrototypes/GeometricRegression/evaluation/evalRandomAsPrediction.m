evaluation = evaluationStruct;
evaluation.name = 'random number in labels range';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);

minLabel = min(regressionInfo.trainingLabels);
maxLabel = max(regressionInfo.trainingLabels);
evaluation.labels = unifrnd(minLabel, maxLabel, nbOfTrainingSamples, 1);
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluations = [evaluations ; evaluation];