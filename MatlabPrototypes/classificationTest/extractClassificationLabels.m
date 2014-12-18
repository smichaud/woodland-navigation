disp('Extracting traversability label (for classification)...');
% This is just to test if classification will give better result than
% regression. You have to compute regression label firsts


nbOfSamples = length(dataset);
traversabilityCosts = zeros(nbOfSamples,1);
for i=1:nbOfSamples
    traversabilityCosts(i) = dataset(i).traversabilityCost;
end

[vector orderedIndexes] = sort(traversabilityCosts);
meanCost = median(traversabilityCosts);

for i=1:nbOfSamples
    if dataset(i).traversabilityCost <= vector(round(nbOfSamples/3))
        dataset(i).traversabilityCost = 0;
    elseif dataset(i).traversabilityCost <= vector(round(2*nbOfSamples/3))
        dataset(i).traversabilityCost = 1;
    else
        dataset(i).traversabilityCost = 2;
    end
end

