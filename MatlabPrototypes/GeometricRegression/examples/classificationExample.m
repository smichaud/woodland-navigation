rng default % Default random seed
 
% [feature1 feature2 label]
trainData = [ ...
    [6,  300,  1];
    [3,  300,  0];
    [8,  300,  1];
    [11, 2000, 0];
    [3,  100,  0];
    [6,  1000, 0];
    ];
 
features = trainData(:,(1:2));
classLabels = trainData(:,3);
 
nbOfTrees = 20;
 
% Train the TreeBagger (Decision Forest). Method: classification/regression
trainedDecisionForest = ...
    TreeBagger(nbOfTrees,features,classLabels, 'Method', 'classification');

testData1 = [7, 300];
predictionData1 = str2double(trainedDecisionForest.predict(testData1))

testData2 = [7, 1500]; 
predictionData2 = str2double(trainedDecisionForest.predict(testData2))

