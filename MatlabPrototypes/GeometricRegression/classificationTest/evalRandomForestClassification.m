disp('Evaluating random forest with all features (classification)...');

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
        'Method', 'classification',...
        'MinLeaf', regressionInfo.nbOfLeaves);
    
    % Output is a string
    leaveOneOutPrediction(i) = str2double(...
        regressor.predict(regressionInfo.trainingFeatures(i,:)));
end

evaluation.labels = leaveOneOutPrediction;

classificationResults = ...
    size(find(evaluation.labels == regressionInfo.trainingLabels),1)/...
    size(evaluation.labels,1)

