nbOfTrees = 500; % input('Please choose the number of tree used');
nbOfLeaves = 20; % input('Number of leaves');

nbOfSamples = length(dataset);
leaveOneOutPrediction = zeros(nbOfSamples, 1);
for i = 1:nbOfSamples
    otherIndexes = 1:nbOfSamples;
    otherIndexes(:,i) = [];
    
    regressor = TreeBagger(...
        nbOfTrees,...
        regressorFeatures(otherIndexes,:),...
        regressorLabels(otherIndexes,:),...
        'Method', 'R',...
        'MinLeaf', nbOfLeaves);
    
    leaveOneOutPrediction(i) =...
        regressor.predict(regressorFeatures(i,:));
    
    disp(sprintf('%f: %f -> %f',i,leaveOneOutPrediction(i),regressorLabels(i)));
end

hold on;
plot(1:nbOfSamples, leaveOneOutPrediction(i), 'b')