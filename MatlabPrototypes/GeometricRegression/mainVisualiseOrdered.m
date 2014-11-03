labelsToOrder = zeros(length(dataset),1);
for i = 1:length(dataset)
    labelsToOrder(i) = dataset(i).traversabilityCost;
end

[orderedVector,orderedIndexes] = sort(labelsToOrder);

orderedDataset = dataset(orderedIndexes);

% examplesToRemove = {};
for i = 1:length(orderedDataset)
    showSample(orderedDataset(i), areaOfInterest);
    uiwait;
%     if strcmp(questdlg('Keep last example', '','Yes','No','Yes'),'No')
%         examplesToRemove{length(examplesToRemove)+1} =...
%             orderedDataset(i).name;
%     end
end