function showSingleDataFullIMU(data, areaOfInterest, traversabilityCostInfo)
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
xMin = areaOfInterest.distFromRobot;
xMax = areaOfInterest.distFromRobot + areaOfInterest.depth;
set(gca, 'xlim', [xMin xMax]);
yMin = -areaOfInterest.width/2;
yMax = areaOfInterest.width/2;
set(gca, 'ylim', [yMin yMax]);
zMin = data.groundHeight + areaOfInterest.groundThreshold;
zMax = data.groundHeight + areaOfInterest.height;
set(gca, 'zlim', [zMin zMax]);

subplot(1,3,3)
hold on;
title(sprintf('X Inertia (traversability cost : %f)',...
    data.traversabilityCost));
plot(data.rawInertia(:,1), data.rawInertia(:,2), 'r');
set(gca, 'ylim', [-0.8 0.8]);

start = data.traversabilityStartTime;
stop = data.traversabilityStopTime;
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', 4);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', 4);

end

