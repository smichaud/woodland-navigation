nbOfSamples = length(dataset);

usedIMU = [2,3,4,5,6,7]; % 2:7 -> inertia x/y/z angularSpeed x/y/z
nbOfFeatures = length(dataset(1).dftIMU(:,1));
% 1:4 = frequencies < 1/4.2 Hz... endFractionRemoved:end > 5 Hz
endFractionRemoved = 1/2;
ignoredFeatures = [1 round(endFractionRemoved*nbOfFeatures):nbOfFeatures];

nbOfColumnsMerged = 1; % 1 won't merge any columns

areVectorsNormalized = false;
areFeaturesNormalized = false;
isMatrixWhiten = false;

nbOfClusters = 3;
minkowskiDistance = 1;

dftVectors = [];
for imuIndex = 1:length(usedIMU)
    partialDftVectors = zeros(nbOfSamples, nbOfFeatures);
    for i = 1:nbOfSamples
        partialDftVectors(i,:) = dataset(i).dftIMU(:,usedIMU(imuIndex))';
    end
    partialDftVectors(:,ignoredFeatures) = [];

    partialDftVectors = sumSubColumns(...
        partialDftVectors, nbOfColumnsMerged);
    
    if areVectorsNormalized
        minValue = min(partialDftVectors(:));
        maxValue = max(partialDftVectors(:));
        partialDftVectors = ...
            (partialDftVectors-minValue)./(maxValue-minValue);
    end  
    
    dftVectors = [dftVectors partialDftVectors];
end

if areFeaturesNormalized
    for i = 1:size(dftVectors, 2)
        minValue = min(dftVectors(:,i));
        maxValue = max(dftVectors(:,i));
        dftVectors(:,i) = (dftVectors(:,i)-minValue)./(maxValue-minValue);
    end
end

if isMatrixWhiten
    dftVectors = whitenMatrix(dftVectors);
end