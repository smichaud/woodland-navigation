nbOfSamples = length(dataset);
for i=1:nbOfSamples
    clf;
    close all;
    figure('Name', dataset(i).name, 'units','normalized','outerposition',[0 0 1 1])
    
    subplot(1,3,1);
    imshow(dataset(i).image);
    title(sprintf('Image of %s',dataset(i).name));
    
    subplot(1,3,2);
    scatter3(dataset(i).areaOfInterest(:,1),...
        dataset(i).areaOfInterest(:,2),...
        dataset(i).areaOfInterest(:,3), '.');
    title('Area of interest (point cloud)');
    
    subplot(1,3,3)
    hold on;
    title(sprintf('Motor currents (traversability cost : %f)', dataset(i).traversabilityCost));
    plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,2), 'b');
    plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,3), 'r');
    
    start = dataset(i).rawCurrents(dataset(i).traversabilityStartIndex,1);
    stop = dataset(i).rawCurrents(dataset(i).traversabilityStopIndex,1);
    line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', 4);
    line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', 4);
    uiwait;
end


