featureNames = {'density', 'highestPoint'};
nbOfFeatures = size(featureNames,2);
dataset(i).features = containers.Map(featureNames,repmat([],nbOfFeatures));

% Set the features template for all samples
featuresTemplate = featureStruct;
for i=1:nbOfFeatures
    featuresTemplate(i).name = featureNames{i};
end
for i=1:nbOfSamples
    dataset(i).features = featuresTemplate;
end


% Compute the features
densityIndex = find(strcmp(featureNames,'density'));
highestPointIndex = find(strcmp(featureNames,'highestPoint'));
volume = areaOfInterest.width * areaOfInterest.height ...
    * areaOfInterest.depth;
for i=1:nbOfSamples
    nbOfPoints = size(dataset(i).areaOfInterest,1);
    dataset(i).features(densityIndex).values = nbOfPoints/volume;
    
    dataset(i).features(highestPointIndex).values = ...
        max(dataset(i).areaOfInterest(:,3));
end
