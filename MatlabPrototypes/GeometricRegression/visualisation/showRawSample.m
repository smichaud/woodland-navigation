function userTraversabilityCost = showRawSample(data, areaOfInterest,...
    askUserTraversabilityCost)

% To interpolate variance/integral 
start = 30;
step = 0.01;
stop = 38;

subplot(3,4,[1:2 5:6]);
imshow(data.image);
title(sprintf('Image of %s',data.name));

subplot(3,4,9);
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
axis equal;

subplot(3,4,10);
cla;
hold on;
plot(data.rawCurrents(:,1), data.rawCurrents(:,2), 'b');
plot(data.rawCurrents(:,1), data.rawCurrents(:,3), 'r');

interpolatedVariance1 =  interpolateVariance(data.rawCurrents(:,1),...
    data.rawCurrents(:,2), start, step, stop);
interpolatedVariance2 =  interpolateVariance(data.rawCurrents(:,1),...
    data.rawCurrents(:,3), start, step, stop);

interpolatedIntegral1 = interpolateIntegral(data.rawCurrents(:,1),...
    data.rawCurrents(:,2), start, step, stop);
interpolatedIntegral2 = interpolateIntegral(data.rawCurrents(:,1),...
    data.rawCurrents(:,3), start, step, stop);

title(sprintf('Motor current (Variance : %f, Integral : %f)',...
    interpolatedVariance1 + interpolatedVariance2, ...
    interpolatedIntegral1 + interpolatedIntegral2));
set(gca, 'ylim', [0 20]);

subplot(3,4,3);
plot(data.rawIMU(:,1), data.rawIMU(:,2), 'r');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,2), start, step, stop);
title(sprintf('X inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);

subplot(3,4,7);
plot(data.rawIMU(:,1), data.rawIMU(:,3), 'g');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,3), start, step, stop);
title(sprintf('Y inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);

subplot(3,4,11);
plot(data.rawIMU(:,1), data.rawIMU(:,4), 'b');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,4), start, step, stop);
title(sprintf('Z inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [0.8 1.5]);

subplot(3,4,4);
plot(data.rawIMU(:,1), data.rawIMU(:,5), 'r');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,5), start, step, stop);
title(sprintf('X angular speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.6 0.6]);

subplot(3,4,8);
plot(data.rawIMU(:,1), data.rawIMU(:,6), 'g');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,6), start, step, stop);
title(sprintf('Y angular speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);

subplot(3,4,12);
plot(data.rawIMU(:,1), data.rawIMU(:,7), 'b');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,7), start, step, stop);
title(sprintf('Z angular speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.2 0.2]);

if askUserTraversabilityCost == true
    prompt = 'Intuitive traversability cost (1 to 10): ';
    userTraversabilityCost = input(prompt);
end

end

