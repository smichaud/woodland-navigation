function predictedLabels = knnRegression(trainFeatures, trainLabels,...
    featuresToPredict, K, minkowskiDist)

[indexes distances] = knnsearch(trainFeatures, featuresToPredict,...
    'K', K, 'Distance', 'minkowski', 'P', minkowskiDist);

nbOfDistNull = length(find(distances == 0));
if nbOfDistNull == 0
    distanceInverses = 1./distances;
    normalisationFactor = 1/sum(distanceInverses);
    knnWeights = (normalisationFactor*distanceInverses)';
else
    indexes = indexes(1:nbOfDistNull);
    knnWeights = repmat(1/nbOfDistNull, 1, nbOfDistNull)';
end

nbOfPredictions = size(featuresToPredict,1);
predictionsDimension = size(trainLabels, 2);
predictedLabels = zeros(nbOfPredictions, predictionsDimension);
for i = 1:nbOfPredictions
    repKnnWeights = repmat(knnWeights,1,predictionsDimension);
    knnLabels = trainLabels(indexes,:);
    weightedKnn = repKnnWeights.*knnLabels;
    predictedLabels(i,:) = sum(weightedKnn,1);
end

end
