function predictedLabels = knnRegression(trainFeatures, trainLabels,...
    featuresToPredict, K, minkowskiDist)

trainFeatures = [1 1; 2 2; 3 3; 4 4];
featuresToPredict = [3.3 3.3; 2.2 2.2; 4.4 4.4];
trainLabels = [1 1 1 1 1 ; 2 2 2 2 2 ; 3 3 3 3 3 ; 4 4 4 4 4];

[indexes distances] = knnsearch(trainFeatures, featuresToPredict,...
    'K', K, 'Distance', 'minkowski', 'P', minkowskiDist);

knnWeights = distances./repmat(sum(distances, 2), 1, K);

nbOfPredictions = size(featuresToPredict,1);
predictionsDimension = size(trainLabels, 2);
predictedLabels = zeros(nbOfPredictions, predictionsDimension);
for i = 1:nbOfPredictions
    repKnnWeights = repmat(knnWeights(i,:)',1,predictionsDimension);
    knnLabels = trainLabels(indexes(i,:),:);
    weightedKnn = repKnnWeights.*knnLabels;
    predictedLabels(i,:) = sum(weightedKnn,1);
end

end

