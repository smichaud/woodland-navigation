% Convert my dataset to the TreeBagger format (col=feature, line=sample)
nbOfSamples = length(dataset);
regressorFeatures = [];
regressorLabels = zeros(nbOfSamples,1);

for i = 1:nbOfSamples
    regressorFeatures = [regressorFeatures;
        cell2mat(dataset(i).features.values)];
    regressorLabels(i) = dataset(i).traversabilityCost;
end

% Define a feature names vector
% TODO make sure the order is the same between keys and values... 
% featuresNames = {};
% if nbOfSamples > 0
%     featureKeys = dataset(1).features.keys;
%     featureValues = dataset(1).features.values;
%     index = 1;
%     for i = 1:length(featureValues)
%         endIndex = index + length(featureValues{i}) - 1;
%         featureNames{index:endIndex} = repmat(featureKeys{i},...
%             1, length(featureValues(i)));
%         index = endIndex + 1;
%     end
% end


