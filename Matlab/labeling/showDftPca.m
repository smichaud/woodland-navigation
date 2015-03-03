initLabeling;

data = dftVectors;
nbOfSamples = size(dftVectors,1);
nbOfFeatures = size(dftVectors,2);

[eigenVectors, eigenValues] = eig(cov(data));
[sortedValues,sortIndex] = sort(diag(eigenValues),'descend');

figure;
plot(1:length(sortedValues), sortedValues);
xlabel('Component number');
ylabel('Eigen value')

figure;
plot(1:length(sortedValues), cumsum(sortedValues)./sum(sortedValues));
xlabel('Component number');
ylabel('Proportion of variance')

% Principal components (unit vectors)
nbOfComponents = 3;
pcaVectors = eigenVectors(:,sortIndex(1:nbOfComponents));

% Project the data on the principal components
dataMeanVector = mean(data,2);
projectedData = (pcaVectors'*(data-dataMeanVector*ones(1,nbOfFeatures))')';

figure
if nbOfComponents == 1
    plot(projectedData, 0, 'k.');
    set(gca, 'ylim', [-0.01 0.01]);
elseif nbOfComponents == 2
    plot(projectedData(:,1),projectedData(:,2), '.');
elseif nbOfComponents == 3
    scatter3(projectedData(:,1),projectedData(:,2),projectedData(:,3), '.');
else
    disp('Unable to show over 3D')
end

