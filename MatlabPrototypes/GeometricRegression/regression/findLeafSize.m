% ========== Finding the Optimal Leaf Size
nbOfTrees = 200; % less for fast test, but greater is better (> 100)
leaf = [5 10 20 50 100];
col = 'rbcmy'; % For line colors

figure
for i=1:length(leaf)
    regressor = TreeBagger(nbOfTrees,...
        regressorFeatures, regressorLabels,...
        'Method', 'R',...
        'OOBPred', 'On',...
        'MinLeaf',leaf(i));
    plot(oobError(regressor),col(i));
    hold on;
end
title('Finding the Optimal Leaf Size');
xlabel('Number of Grown Trees');
ylabel('Mean Squared Error');
legend({'5' '10' '20' '50' '100'},'Location','NorthEast');
hold off;
uiwait;