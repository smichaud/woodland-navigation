% ========== Finding the Optimal Leaf Size
disp('Finding the optimal leaf size...');

nbOfTrees = 200; % less for fast test, but greater is better (> 100)
leaf = [5 10 20 50 100];
col = 'rbcmy'; % For line colors

bestLeafScore = Inf;
bestLeafNumber = 5;
meanNbUsed = 75;
figure
for i=1:length(leaf)
    regressor = TreeBagger(nbOfTrees,...
        regressionInfo.trainingFeatures, regressionInfo.trainingLabels,...
        'Method', 'R',...
        'OOBPred', 'On',...
        'MinLeaf',leaf(i));
    currentOobError = oobError(regressor);
    plot(currentOobError,col(i));
    hold on;
    
    if mean(currentOobError(nbOfTrees-meanNbUsed:nbOfTrees,1))...
            < bestLeafScore
        bestLeafNumber = leaf(i);
        bestLeafScore = mean(...
            currentOobError(nbOfTrees-meanNbUsed:nbOfTrees,1));
    end
end
title('Finding the Optimal Leaf Size');
xlabel('Number of Grown Trees');
ylabel('Mean Squared Error');
legend({'5' '10' '20' '50' '100'},'Location','NorthEast');
hold off;

% Automatic version
regressionInfo.nbOfLeaves = bestLeafNumber;

disp([num2str(bestLeafNumber) ' leaves will be used for the random forest']);

% User version
% regressionInfo.nbOfLeaves = str2double(...
%     inputdlg('Desired number of leaves ?','', 1, {'5'}));
% uiwait;