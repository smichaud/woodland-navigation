nbOfSamples = length(dataset);

traversabilityCost = zeros(nbOfSamples, 1);
for i=1:nbOfSamples
%     traversabilityCost(i) = dataset(i).userTraversabilityCost;
    traversabilityCost(i) = dataset(i).traversabilityCost;
end 
[vector orderedIndexes] = sort(traversabilityCost);

clf;
close all;
figure('Name', 'Raw data representation', 'units','normalized',...
    'outerposition',[0 0 1 1])
for i=1:nbOfSamples
    showSample(dataset(orderedIndexes(i)),...
        areaOfInterest, false);
    waitforbuttonpress;
%     uiwait
end
