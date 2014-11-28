nbOfSamples = length(dataset);

kmeansAlgo = 1;
kmedoidsAlgo = 2; % Not in my version
gaussianMixtureAlgo = 3; % n needs to be > p
clusterAlgo = kmeansAlgo;

usedIMU = 6; % 2:7 -> inertia x/y/z angularSpeed x/y/z
nbOfClusters = 8;
imageConfiguration = [3 4]; %
nbOfImagePerFigure = prod(imageConfiguration);

dftVectors = zeros(nbOfSamples, length(dataset(1).dftIMU(:,1)));
for i = 1:nbOfSamples
    dftVectors(i,:) = dataset(i).dftIMU(:,usedIMU)';
    distanceMatrix = squareform(pdist(dftVectors, 'euclidean'));
end

if clusterAlgo == kmeansAlgo
    clustersIndices = kmeans(dftVectors, nbOfClusters);
elseif clusterAlgo == kmedoidsAlgo
    kmedoids(dftVectors, nbOfClusters);
elseif clusterAlgo == gaussianMixtureAlgo
    clustersIndices = ...
        cluster(gmdistribution.fit(dftVectors,nbOfClusters),dftVectors);
end

for k = 1:nbOfClusters
    waitfor(msgbox(sprintf('Cluster %d',k)));
    
    currentClusterIndices = find(clustersIndices == k);
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
