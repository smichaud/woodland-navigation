% Convert my dataset to the neutral network format (col=sample, line=features)
disp('Convert data in preparation for regression (neutral network)...');

nbOfSamples = length(dataset);
nbOfFeatures = length(cell2mat(dataset(1).features.values)');
features = zeros(nbOfFeatures,nbOfSamples);
labels = zeros(1,nbOfSamples);

for i = 1:nbOfSamples
    features(:,i) = cell2mat(dataset(i).features.values)';
    labels(i) = dataset(i).traversabilityCost;
end