disp('Extracting all features...');

% Prevent dataset from sharing the features
nbOfSamples = length(dataset);
for i=1:nbOfSamples
    dataset(i).features = containers.Map;
end

extractDensity;
extractHighestPoint;
extractLayersXY;
extractHistogramXZ;
