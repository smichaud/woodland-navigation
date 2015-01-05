function predictedLabels = knnRegression(trainFeatures, trainLabels,...
    featuresToPredict, K, minkowskiDist)

nbOfPredictions = size(featuresToPredict,1);
predictionsDimension = size(trainLabels, 2);
predictedLabels = zeros(nbOfPredictions, predictionsDimension);

[indexes distances] = knnsearch(trainFeatures, featuresToPredict,...
    'K', K, 'Distance', 'minkowski', 'P', minkowskiDist);

for i = 1:nbOfPredictions
    nbOfDistNull = length(find(distances(i,:) == 0));
    if nbOfDistNull == 0
        distanceInverses = 1./distances(i,:);
        normalisationFactor = 1/sum(distanceInverses);
        knnWeights = (normalisationFactor*distanceInverses)';
        indexesIt = indexes(i,:);
    else
        indexesIt = indexes(i,1:nbOfDistNull);
        knnWeights = repmat(1/nbOfDistNull, 1, nbOfDistNull)';
    end

    repKnnWeights = repmat(knnWeights,1,predictionsDimension);
    knnLabels = trainLabels(indexesIt,:);
    weightedKnn = repKnnWeights.*knnLabels;
    predictedLabels(i,:) = sum(weightedKnn,1);
end

end
