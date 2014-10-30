% Convert my dataset to the TreeBagger format (col=feature, line=sample)
disp('Convert data in preparation for regression...');

trainingSetSize = 0.75;
testSetSize = 1-trainingSetSize;

nbOfSamples = length(dataset);
regressionInfo.trainingFeatures = [];
regressionInfo.testFeatures = [];
regressionInfo.trainingLabels = zeros(...
    round(nbOfSamples*trainingSetSize),1);
regressionInfo.testLabels = zeros(round(nbOfSamples*testSetSize),1);

for i = 1:round(nbOfSamples*trainingSetSize) % Training set
    regressionInfo.trainingFeatures = [regressionInfo.trainingFeatures;
        cell2mat(dataset(i).features.values)];
    regressionInfo.trainingLabels(i) = dataset(i).traversabilityCost;
end

offset = round(nbOfSamples*trainingSetSize);
for i = offset+1:nbOfSamples % Test set
    regressionInfo.testFeatures = [regressionInfo.testFeatures;
        cell2mat(dataset(i).features.values)];
    regressionInfo.testLabels(i-offset) = dataset(i).traversabilityCost;
end

% Define a feature names vector
% TODO make sure the order is the same between keys and values... 
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


