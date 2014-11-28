nbOfSamples = length(dataset)

usedIMU = 6; % 2:7 -> inertia x/y/z angularSpeed x/y/z
nbOfFeatures = length(dataset(1).dftIMU(:,1));
% Ignoring 1 Should remove average angle of the ground
ignoredFeatures = [1 round(nbOfFeatures/2):nbOfFeatures]; 

dftVectors = zeros(nbOfSamples, nbOfFeatures); 
for i = 1:nbOfSamples
	dftVectors(i,:) = dataset(i).dftIMU(:,usedIMU)';	 
end

dftVectors(:,ignoredFeatures) = [];

distanceMatrix = squareform(pdist(dftVectors, 'euclidean'));

for i = 1:nbOfSamples
    [vector orderedIndexes] = sort(distanceMatrix(i,:));
    
    figure('Name', 'Nearest Neighbor (IMU)', 'units','normalized',...
        'outerposition',[0 0 1 1]);
    subplot(2,2,1);
    imshow(dataset(i).image);
    title('Example')
    subplot(2,2,2);
    imshow(dataset(orderedIndexes(2)).image);
    title(sprintf('1st nearest (%f)', vector(2)));
    subplot(2,2,3);
    imshow(dataset(orderedIndexes(3)).image);
    title(sprintf('2nd nearest (%f)', vector(3)));
    subplot(2,2,4);
    imshow(dataset(orderedIndexes(4)).image);
    title(sprintf('3rd nearest (%f)', vector(4)));
    
    waitforbuttonpress;
    close all;
end
