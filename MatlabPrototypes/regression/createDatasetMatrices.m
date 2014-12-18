function [ features labels featureNames ] = ...
    createDatasetMatrices(dataset)

nbOfSamples = length(dataset);
nbOfFeatures = length(cell2mat(dataset(1).features.values));
features = zeros(nbOfSamples, nbOfFeatures);
labels = zeros(nbOfSamples,1);

for i = 1:round(nbOfSamples)
    features(i,:) = [cell2mat(dataset(i).features.values)];
    labels(i) = dataset(i).traversabilityCost;
end

featureNames = {};
if nbOfSamples > 0
    featureKeys = dataset(1).features.keys;
    featureValues = dataset(1).features.values;
    index = 1;
    for i = 1:length(featureValues)
        for j = 1:length(featureValues{i})
            if length(featureValues{i}) > 1                
                featureNames{index} = ...
                    [featureKeys{i},'_',num2str(j)];
            else
                featureNames{index} = featureKeys{i};
            end
            index = index + 1;
        end
    end
end


