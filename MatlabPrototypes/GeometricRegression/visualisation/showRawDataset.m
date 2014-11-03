nbOfSamples = length(dataset);

userTraversabilityCost = zeros(nbOfSamples, 1);
for i=1:nbOfSamples
    userTraversabilityCost(i) = dataset(i).userTraversabilityCost;
end 
[vector orderedIndexes] = sort(userTraversabilityCost);

clf;
close all;
figure('Name', 'Raw data representation', 'units','normalized',...
    'outerposition',[0 0 1 1])
for i=1:nbOfSamples
    showRawSample(dataset(orderedIndexes(i)),...
        areaOfInterest, false);
    waitforbuttonpress;
end
