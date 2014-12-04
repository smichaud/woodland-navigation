function [ projectedData pcaVectors] = getPCA( data, nbOfComponents )

nbOfSamples = size(data,1);
nbOfFeatures = size(data,2);

[eigenVectors, eigenValues] = eig(cov(data));
[sortedValues,sortIndex] = sort(diag(eigenValues),'descend');

pcaVectors = eigenVectors(:,sortIndex(1:nbOfComponents));

dataMeanVector = mean(data,2);
projectedData = (pcaVectors'*(data-dataMeanVector*ones(1,nbOfFeatures))')';

end

