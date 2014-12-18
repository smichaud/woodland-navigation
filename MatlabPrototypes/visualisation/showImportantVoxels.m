% This is just a quick test (very dirty coding)
% It will help to show the position of the important voxel of the voxel map

nbOfShownVoxels = 10; 
voxelSide = 0.2000;
[v bestFeaturesIndexes] = sort(regressionInfo.featuresImportance);


nbOfSamples = length(dataset);
for i = 1:nbOfTrainingSamples
    clf;
    close all;
    figure('Name', 'Area of interest with important voxels',...
        'units','normalized',...
        'outerposition',[0 0 1 1]);
    
    scatter3(dataset(i).areaOfInterest(:,1),...
        dataset(i).areaOfInterest(:,2),...
        dataset(i).areaOfInterest(:,3), '.');
    title('Point cloud with most important voxels');
    xMin = areaOfInterest.distFromRobot;
    xMax = areaOfInterest.distFromRobot + areaOfInterest.depth;
    set(gca, 'xlim', [xMin xMax]);
    yMin = -areaOfInterest.width/2;
    yMax = areaOfInterest.width/2;
    set(gca, 'ylim', [yMin yMax]);
    zMin = dataset(i).groundHeight + areaOfInterest.groundThreshold;
    zMax = dataset(i).groundHeight + areaOfInterest.height;
    set(gca, 'zlim', [zMin zMax]);    
    axis equal;
    
    for j = 1:nbOfShownVoxels
        center = tempCubeInfo{i}(bestFeaturesIndexes(j), :);
        impColor = 1 - ((j-1)/(nbOfShownVoxels-1));
        drawCube(center, side, [impColor 1-impColor 0]);
    end
    
    axis([xMin xMax yMin yMax zMin zMax]);
    hold on;    
    uiwait;
end
