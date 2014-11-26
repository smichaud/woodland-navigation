labelsToOrder = zeros(length(dataset),1);
for i = 1:length(dataset)
    labelsToOrder(i) = dataset(i).traversabilityCost;
end

[orderedVector,orderedIndexes] = sort(labelsToOrder);

orderedDataset = dataset(orderedIndexes);

for i = 1:length(orderedDataset)
    showSample(orderedDataset(i), areaOfInterest, false);
    waitforbuttonpress;
end