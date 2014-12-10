% Choose depending on the features used
% featuresIndices = bestFeaturesIndexes;
featuresIndices = 1:length(regressionInfo.featureNames);

regressor = TreeBagger(...
    regressionInfo.nbOfTrees,...
    regressionInfo.trainingFeatures(:,featuresIndices),...
    regressionInfo.trainingLabels,...
    'Method', 'R',...
    'MinLeaf', regressionInfo.nbOfLeaves);

%==========================================================================

% Training results
trainingEvaluation = evaluationStruct;

nbOfSamples = length(regressionInfo.trainingLabels);
trainingEvaluation.labels = zeros(nbOfSamples, 1);
for i = 1:nbOfSamples
    trainingEvaluation.labels(i) = regressor.predict(...
        regressionInfo.trainingFeatures(i,featuresIndices));
end

trainingEvaluation.meanSquaredError = ...
    mean((trainingEvaluation.labels - regressionInfo.trainingLabels).^2);

trainingEvaluation.rSquared = rSquared(regressionInfo.trainingLabels, ...
    trainingEvaluation.labels);

trainingEvaluation


%==========================================================================


% Test results
testEvaluation = evaluationStruct;

nbOfSamples = length(regressionInfo.testLabels);
testEvaluation.labels = zeros(nbOfSamples, 1);
for i = 1:nbOfSamples
    testEvaluation.labels(i) = regressor.predict(...
        regressionInfo.testFeatures(i,featuresIndices));
end

testEvaluation.meanSquaredError = ...
    mean((testEvaluation.labels - regressionInfo.testLabels).^2);

testEvaluation.rSquared = rSquared(regressionInfo.testLabels, ...
    testEvaluation.labels);

testEvaluation