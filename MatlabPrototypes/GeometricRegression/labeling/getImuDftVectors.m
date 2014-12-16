function imuDftVectors = getImuDftVectors(dataset, params)

nbOfSamples = length(dataset);
nbOfFeatures = length(dataset(1).dftIMU(:,1));

ignoredFeatures = [];
if params.isFirstIndexRemoved
    ignoredFeatures = [1];
end
ignoredFeatures = [ignoredFeatures...
    round(params.endFractionRemoved*nbOfFeatures):nbOfFeatures];

imuDftVectors = [];
for imuIndex = 1:length(params.usedIMU)
    partialDftVectors = zeros(nbOfSamples, nbOfFeatures);
    for i = 1:nbOfSamples
        partialDftVectors(i,:) = dataset(i).dftIMU(:,...
            params.usedIMU(imuIndex))';
    end
    partialDftVectors(:,ignoredFeatures) = [];

    partialDftVectors = sumSubColumns(...
        partialDftVectors, params.nbOfColumnsMerged);
    
    if params.areVectorsNormalized
        minValue = min(partialDftVectors(:));
        maxValue = max(partialDftVectors(:));
        partialDftVectors = ...
            (partialDftVectors-minValue)./(maxValue-minValue);
    end  
    
    imuDftVectors = [imuDftVectors partialDftVectors];
end

if params.areFeaturesNormalized
    for i = 1:size(imuDftVectors, 2)
        minValue = min(imuDftVectors(:,i));
        maxValue = max(imuDftVectors(:,i));
        imuDftVectors(:,i) = ...
            (imuDftVectors(:,i)-minValue)./(maxValue-minValue);
    end
end

if params.isMatrixWhiten
    imuDftVectors = whitenMatrix(imuDftVectors);
end

end

