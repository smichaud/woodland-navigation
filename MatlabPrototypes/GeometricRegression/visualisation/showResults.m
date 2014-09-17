densities = [];
highestPoints = [];
traversabilityCosts = [];

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    densities = [densities dataset(i).features('density')];
    highestPoints = [highestPoints dataset(i).features('highestPoint')];
    traversabilityCosts = [traversabilityCosts ...
        dataset(i).traversabilityCost];
end

clf;
close all;
figure('Name', 'Results', 'units','normalized','outerposition',[0 0 1 1])

subplot(1,2,1);
plot(densities, traversabilityCosts, '*');
title('Density effect on traversability cost ');

subplot(1,2,2);
plot(highestPoints, traversabilityCosts, '*');
title('Max height effect on traversability cost ');
