% ========== Finding the Optimal Leaf Size
disp('Producing the figure to find optimal leaf size...');

nbOfTrees = 200; % less for fast test, but greater is better (> 100)
leaf = [5 10 20 50 100];
col = 'rbcmy'; % For line colors

figure
for i=1:length(leaf)
    regressor = TreeBagger(nbOfTrees,...
        regressionInfo.features, regressionInfo.labels,...
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

regressionInfo.nbOfLeaves = str2double(...
    inputdlg('Desired number of leaves ?','', 1, {'5'}));

uiwait;