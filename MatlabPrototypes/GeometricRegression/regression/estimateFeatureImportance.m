% ========== Estimating Feature Importance
nbOfTrees = 200;
nbOfLeaves = 5; % Should be the best value from the previous step

regressor = TreeBagger(nbOfTrees,...
    regressionInfo.features,regressionInfo.labels,...
    'Method','R',...
    'OOBVarImp','On',...
    'MinLeaf',nbOfLeaves);

% Check everything is still ok
% figure
% plot(oobError(regressor));
% title('Error Curve (still ok ?)');
% xlabel('Number of Grown Trees');
% ylabel('Out-of-Bag Mean Squared Error');
% uiwait;

% Importance histogram
figure
bar(regressor.OOBPermutedVarDeltaError);
title('Feature Importance');
xlabel('Feature Number');
ylabel('Out-of-Bag Feature Importance');
idxvar = find(regressor.OOBPermutedVarDeltaError > 0.7); % threshold = 0.7
uiwait;