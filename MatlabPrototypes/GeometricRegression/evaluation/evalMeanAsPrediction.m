result = resultStruct;
result.name = 'dataset mean';

nbOfSamples = length(dataset);
leaveOneOutPrediction = zeros(nbOfSamples, 1);

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
        regressor.predict(regressionInfo.features(i,:));
end

result.labels = leaveOneOutPrediction;
result.meanSquaredError = mean((result.labels - regressionInfo.labels).^2);

results = [results;
           result];

hold on;
plot(1:nbOfSamples, regressionInfo.labels, '*r')
plot(1:nbOfSamples, leaveOneOutPrediction, '*b')
title(['Result using the ' result.name]);
xlabel('Index of the sample');
ylabel('Traversability cost');
legend({'Measured' 'Predicted'},'Location','EastOutside');