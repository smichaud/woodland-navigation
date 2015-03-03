disp('Evaluating neutral network with all features...');

best_rSquared = -100000;
best_hiddenLayerSize = 0;

nbFeaturesUsed = 5;
[v bestFeaturesIndexes] = sort(regressionInfo.featuresImportance);
featuresIndice = bestFeaturesIndexes(1:nbFeaturesUsed);

for hiddenLayerSize = 2:25

    nbOfTrainingSamples = length(labels);
    leaveOneOutPredictions = zeros(1,nbOfTrainingSamples);
    for i = 1:nbOfTrainingSamples
        otherIndexes = 1:nbOfTrainingSamples;
        otherIndexes(:,i) = [];

        net = fitnet(hiddenLayerSize);
%         net.trainFcn = 'trainbr';
        net.trainParam.showWindow = 0;
        [net,tr] = train(net,features(featuresIndice,otherIndexes),...
            labels(:,otherIndexes));  

        leaveOneOutPredictions(i) = net(features(featuresIndice,i));
    end

    meanSquaredError = mean((labels - leaveOneOutPredictions).^2)
    rSquaredResult = rSquared(labels', leaveOneOutPredictions')
    
    if rSquaredResult > best_rSquared
        best_hiddenLayerSize = hiddenLayerSize;
        best_rSquared = rSquaredResult;
    end
end

best_rSquared