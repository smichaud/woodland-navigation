function showSingleData(data)
    clf;
    close all;
    figure('Name', data.name, 'units','normalized',...
        'outerposition',[0 0 1 1])
    
    subplot(1,3,1);
    imshow(data.image);
    title(sprintf('Image of %s',data.name));
    
    subplot(1,3,2);
    scatter3(data.areaOfInterest(:,1),...
        data.areaOfInterest(:,2),...
        data.areaOfInterest(:,3), '.');
    title('Area of interest (point cloud)');
    
    subplot(1,3,3)
    hold on;
    title(sprintf('Motor currents (traversability cost : %f)',...
        data.traversabilityCost));
    plot(data.rawCurrents(:,1), data.rawCurrents(:,2), 'b');
    plot(data.rawCurrents(:,1), data.rawCurrents(:,3), 'r');
    
    start = data.rawCurrents(data.traversabilityStartIndex,1);
    stop = data.rawCurrents(data.traversabilityStopIndex,1);
    line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', 4);
    line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', 4);
end

