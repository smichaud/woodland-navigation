nbOfSamples = length(dataset);

usedIMU = [2:7]; % 2:7 -> inertia x/y/z angularSpeed x/y/z
nbOfFeatures = length(dataset(1).dftIMU(:,1));
% Ignoring 1 Should remove average angle of the ground
ignoredFeatures = [1 round(1*nbOfFeatures/3):nbOfFeatures];
areFeaturesNormalized = false;

nbOfClusters = 8;

dftVectors = [];
for imuIndex = 1:length(usedIMU)    
    partialDftVectors = zeros(nbOfSamples, nbOfFeatures);
    for i = 1:nbOfSamples
        partialDftVectors(i,:) = dataset(i).dftIMU(:,usedIMU(imuIndex))';
    end
    partialDftVectors(:,ignoredFeatures) = [];
    
    dftVectors = [dftVectors partialDftVectors];
end
size(dftVectors)

if areFeaturesNormalized
    for i = 1:size(partialDftVectors, 2)
        minValue = min(partialDftVectors(:,i));
        maxValue = max(partialDftVectors(:,i));
        partialDftVectors(:,i) = (partialDftVectors(:,i)-minValue)./(maxValue-minValue);
    end
end

