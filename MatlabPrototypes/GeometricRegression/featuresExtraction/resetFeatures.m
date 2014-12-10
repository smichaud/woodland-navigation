if isfield(regressionInfo, 'featureNames')
    regressionInfo = rmfield(regressionInfo, 'featureNames');
end
if isfield(dataset, 'features')
    dataset = rmfield(dataset,'features');
end
% Prevent dataset from sharing the features
nbOfSamples = length(dataset);
for i=1:nbOfSamples    
    dataset(i).features = containers.Map;
end