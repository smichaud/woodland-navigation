initLabeling;

kmeansAlgo = 1;
kmedoidsAlgo = 2; % Not in my version
gaussianMixtureAlgo = 3; % n needs to be > p
clusterAlgo = kmeansAlgo;

imageConfiguration = [3 4];
nbOfImagePerFigure = prod(imageConfiguration);

if clusterAlgo == kmeansAlgo
    clustersIndices = kmeans(dftVectors, nbOfClusters,...
        'distance', 'cityblock');
elseif clusterAlgo == kmedoidsAlgo
    kmedoids(dftVectors, nbOfClusters);
elseif clusterAlgo == gaussianMixtureAlgo
    clustersIndices = ...
        cluster(gmdistribution.fit(dftVectors,nbOfClusters),dftVectors);
end

clusterDistFromOrigin = zeros(nbOfClusters,1);
for k = 1:nbOfClusters
    currentClusterIndices = find(clustersIndices == k);
    clusterDistFromOrigin(k) = ...
        norm(mean(dftVectors(currentClusterIndices,:)));
end
[sortedValues,sortedIndexes] = sort(clusterDistFromOrigin);

for k = 1:nbOfClusters
    waitfor(msgbox(sprintf('Cluster %d, mean dist: %f',k, sortedValues(k))));
    
    currentClusterIndices = find(clustersIndices == sortedIndexes(k));
    for i = 1:nbOfImagePerFigure:length(currentClusterIndices)
        figure('Name', 'K-Means from IMU data', 'units','normalized',...
            'outerposition',[0 0 1 1]);
        
        subI = 0;
        while (i+subI <= length(currentClusterIndices) ...
                && subI < nbOfImagePerFigure)
            subI = subI + 1;
            subplot(imageConfiguration(1),imageConfiguration(2),subI);
            imshow(dataset(currentClusterIndices(i-1+subI)).image);
        end
        
        waitforbuttonpress;
        close all;
    end
end
