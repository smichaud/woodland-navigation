evaluation = evaluationStruct;
evaluation.name = 'robust fit with density only';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPredictions = zeros(nbOfTrainingSamples, 1);

densityIndex = find(ismember(regressionInfo.featureNames,'density'));

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    % y = b(2)x + b(1)
    b = robustfit(...
        regressionInfo.trainingFeatures(otherIndexes,densityIndex),...
        regressionInfo.trainingLabels(otherIndexes),...
        'bisquare', 3.2); % param looks better to me (default 4.685)
    
    leaveOneOutPredictions(i) = ...
        b(2)*regressionInfo.trainingFeatures(i, densityIndex) + b(1);
end

evaluation.labels = leaveOneOutPredictions;
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluations = [evaluations ; evaluation];