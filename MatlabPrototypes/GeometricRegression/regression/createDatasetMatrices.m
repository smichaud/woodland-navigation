function [ features labels featureNames ] = ...
    createDatasetMatrices(dataset)

nbOfSamples = length(dataset);
regressionInfo.trainingFeatures = [];
regressionInfo.testFeatures = [];
regressionInfo.trainingLabels = zeros(...
    round(nbOfSamples*trainingSetSize),1);
regressionInfo.testLabels = zeros(round(nbOfSamples*testSetSize),1);

for i = 1:round(nbOfSamples)
    regressionInfo.trainingFeatures = [regressionInfo.trainingFeatures;
        cell2mat(dataset(i).features.values)];
    regressionInfo.trainingLabels(i) = dataset(i).traversabilityCost;
end

if nbOfSamples > 0
    featureKeys = dataset(1).features.keys;
    featureValues = dataset(1).features.values;
    index = 1;
    for i = 1:length(featureValues)
        for j = 1:length(featureValues{i})
            if length(featureValues{i}) > 1                
                regressionInfo.featureNames{index} = ...
                    [featureKeys{i},'_',num2str(j)];
            else
                regressionInfo.featureNames{index} = featureKeys{i};
            end
            index = index + 1;
        end
    end
end


