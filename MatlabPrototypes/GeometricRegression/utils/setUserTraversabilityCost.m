nbOfSamples = length(dataset);

for i=1:nbOfSamples
    dataset(i).userTraversabilityCost = showRawSample(dataset(i), ...
        areaOfInterest, true);
end