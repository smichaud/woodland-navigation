nbOfSamples = length(dataset);

for i=1:nbOfSamples
    dataset(i).userTraversabilityCost = showSample(dataset(i), ...
        areaOfInterest, true);
end