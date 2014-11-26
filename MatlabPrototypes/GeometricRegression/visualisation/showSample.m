function userTraversabilityCost = showSample(data, areaOfInterest,...
    askUserTraversabilityCost)

% To interpolate variance/integral
step = 0.01;
start = data.traversabilityStartTime;
stop = data.traversabilityStopTime;
vLineWidth = 2;

clf;
close all;

% ===== Left window =======================================================
figure('Name', data.name, 'units','normalized',...
    'outerposition',[0 0 1 1])

subplot(3,4,[1:2 5:6]);
imshow(data.image);
title(sprintf('Image of %s',data.name));

subplot(3,4,[3:4 7:8]);
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

subplot(3,4,9:12);
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
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

if askUserTraversabilityCost == true
    prompt = 'Intuitive traversability cost (1 to 10): ';
    userTraversabilityCost = input(prompt);
end


% ===== Right window ======================================================
figure('Name', data.name, 'units','normalized',...
    'outerposition',[0 0 2 1])

subplot(3,4,1);
plot(data.rawIMU(:,1), data.rawIMU(:,2), 'r');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,2), start, step, stop);
title(sprintf('X Inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,2);
plot(data.dftIMU(:,1), data.dftIMU(:,2), 'r');
title('Single-Sided Amplitude Spectrum of X Inertia');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.1]);

subplot(3,4,5);
plot(data.rawIMU(:,1), data.rawIMU(:,3), 'g');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,3), start, step, stop);
title(sprintf('Y Inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,6);
plot(data.dftIMU(:,1), data.dftIMU(:,3), 'g');
title('Single-Sided Amplitude Spectrum of Y Inertia');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.06]);

subplot(3,4,9);
plot(data.rawIMU(:,1), data.rawIMU(:,4), 'b');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,4), start, step, stop);
title(sprintf('Z Inertia (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [0.8 1.5]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,10);
plot(data.dftIMU(:,1), data.dftIMU(:,4), 'b');
title('Single-Sided Amplitude Spectrum of Z Inertia');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.25]);

subplot(3,4,3);
plot(data.rawIMU(:,1), data.rawIMU(:,5), 'r');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,5), start, step, stop);
title(sprintf('X Angular Speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.6 0.6]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,4);
plot(data.dftIMU(:,1), data.dftIMU(:,5), 'r');
title('Single-Sided Amplitude Spectrum of X Angular Speed');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.1]);

subplot(3,4,7);
plot(data.rawIMU(:,1), data.rawIMU(:,6), 'g');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,6), start, step, stop);
title(sprintf('Y Angular Speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.8 0.8]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,8);
plot(data.dftIMU(:,1), data.dftIMU(:,6), 'g');
title('Single-Sided Amplitude Spectrum of Y Angular Speed');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.1]);

subplot(3,4,11);
plot(data.rawIMU(:,1), data.rawIMU(:,7), 'b');
interpolatedVariance =  interpolateVariance(data.rawIMU(:,1),...
    data.rawIMU(:,7), start, step, stop);
title(sprintf('Z Angular Speed (Variance : %f)', interpolatedVariance));
set(gca, 'ylim', [-0.2 0.2]);
line([start, start],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);
line([stop stop],ylim, 'Color', [0 0 0], 'LineWidth', vLineWidth);

subplot(3,4,12);
plot(data.dftIMU(:,1), data.dftIMU(:,6), 'b');
title('Single-Sided Amplitude Spectrum of Z Angular Speed');
xlabel('Frequency (Hz)');
ylabel('|Y(f)|');
set(gca, 'ylim', [0 0.1]);

end