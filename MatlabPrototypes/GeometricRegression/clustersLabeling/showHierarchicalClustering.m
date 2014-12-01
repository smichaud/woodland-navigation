imageConfiguration = [3 4]; %
nbOfImagePerFigure = prod(imageConfiguration);

linkageResult = linkage(dftVectors, 'single', 'euclidean');
dendrogram(linkageResult);
clustersIndices = cluster(linkageResult, 'maxclust', 5);

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