nbOfSamples = length(dataset);
leaveOneOutPrediction = zeros(nbOfSamples, 1);

result = resultStruct;
result.name = 'Mean'

for i = 1:nbOfSamples
    otherIndexes = 1:nbOfSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(...
        regressionInfo.nbOfTrees,...
        regressionInfo.features(otherIndexes,:),...
        regressionInfo.labels(otherIndexes,:),...
        'Method', 'R',...
        'MinLeaf', regressionInfo.nbOfLeaves);
    
    leaveOneOutPrediction(i) =...
        regressor.predict(regressorFeatures(i,:));
end

result.labels = leaveOneOutPrediction;

hold on;
plot(1:nbOfSamples, leaveOneOutPrediction(i), 'b')
plot(1:nbOfSamples, regressionInfo.labels(i), 'r')
