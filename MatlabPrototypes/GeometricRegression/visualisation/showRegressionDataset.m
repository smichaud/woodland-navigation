nbOfSamples = length(dataset);

traversabilityCostVector = zeros(nbOfSamples,1);
for i=1:nbOfSamples
   traversabilityCostVector(i) = dataset(i).traversabilityCost;
end
[vector orderedIndexes] = sort(traversabilityCostVector);

for i=1:nbOfSamples
    showRegressionSample(dataset(orderedIndexes(i)), ...
        areaOfInterest, ...
        traversabilityCostInfo);
    uiwait;
end
