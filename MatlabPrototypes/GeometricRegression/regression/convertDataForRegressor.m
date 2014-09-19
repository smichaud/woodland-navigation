% Convert my dataset to the TreeBagger format (col=feature, line=sample)
disp('Convert data to be usable by the regressor...');

nbOfSamples = length(dataset);
regressionInfo.labels = zeros(nbOfSamples,1);

for i = 1:nbOfSamples
    regressionInfo.features = [regressionInfo.features;
        cell2mat(dataset(i).features.values)];
    regressionInfo.labels(i) = dataset(i).traversabilityCost;
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


