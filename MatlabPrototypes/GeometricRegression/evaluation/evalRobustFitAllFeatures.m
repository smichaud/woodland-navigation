disp('Evaluating robust fit with all features...');

evaluation = evaluationStruct;
evaluation.name = 'robust fit with all features';

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
leaveOneOutPredictions = zeros(nbOfTrainingSamples, 1);

for i = 1:nbOfTrainingSamples
    otherIndexes = 1:nbOfTrainingSamples;
    otherIndexes(:,i) = [];
    
    % y = b(2)x + b(1)
    b = robustfit(...
        regressionInfo.trainingFeatures(otherIndexes,:),...
        regressionInfo.trainingLabels(otherIndexes),...
        'bisquare', 3.2); % param looks better to me (default 4.685)

    leaveOneOutPredictions(i) = ...
        regressionInfo.trainingFeatures(i, :)*b(2:end) + b(1);
end

evaluation.labels = leaveOneOutPrediction;
evaluation.meanSquaredError = ...
    mean((evaluation.labels - regressionInfo.trainingLabels).^2);

evaluation.rSquared = rSquared(regressionInfo.trainingLabels, ...
    evaluation.labels);

evaluations = [evaluations ; evaluation];