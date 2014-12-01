data = dftVectors;
nbOfSamples = size(dftVectors,1);
nbOfFeatures = size(dftVectors,2);

[eigenVectors, eigenValues] = eig(cov(data));
[sortedValues,sortIndex] = sort(diag(eigenValues),'descend');

% Principal components (unit vectors)
nbOfComponents = 3;
pcaVectors = eigenVectors(:,sortIndex(1:nbOfComponents)); 

% Project the data on the principal components
dataMeanVector = mean(data,2);
projectedData = (pcaVectors'*(data-dataMeanVector*ones(1,nbOfFeatures))')';

scatter3(projectedData(:,1),projectedData(:,2),projectedData(:,3));
