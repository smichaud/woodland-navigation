% Check everything is still ok
% figure
% plot(oobError(regressor));
% title('Error Curve (still ok ?)');
% xlabel('Number of Grown Trees');
% ylabel('Out-of-Bag Mean Squared Error');
% uiwait;

% Importance histogram
clf;
close all;
figure('Name', 'Feature Importance', 'units','normalized',...
    'outerposition',[0 0 1 1])
bar(regressionInfo.featuresImportance);
title('Feature Importance');
set(gca, 'XTickLabel', regressionInfo.featureNames);
ylabel('Out-of-Bag Feature Importance');
% idxvar = find(regressor.OOBPermutedVarDeltaError > 0.7); % threshold = 0.7
hold off;
% uiwait; % Just put for user interface