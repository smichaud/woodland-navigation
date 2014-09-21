% Start by showing 1 graph per result (real label vs prediction)
nbOfEvaluations = length(evaluations);
nbOfTrainingSamples = length(regressionInfo.trainingLabels);

clf;
close all;
figure('Name', 'Evaluation figure', 'units','normalized',...
    'outerposition',[0 0 1 1])
hold on;

[vector orderedIndexes] = sort(regressionInfo.trainingLabels);

for i = 1:nbOfEvaluations
    subplot(3,2,i);
    hold on;
    plot(1:nbOfTrainingSamples, ...
        regressionInfo.trainingLabels(orderedIndexes), 'or')
    plot(1:nbOfTrainingSamples, ...
        evaluations(i).labels(orderedIndexes), 'b*')
    evalTitle = ['Predicted, using ' evaluations(i).name];
    evalTitle = [evalTitle ' (MSE = ' ];
    evalTitle = [evalTitle num2str(evaluations(i).meanSquaredError)];
    evalTitle = [evalTitle ')'];
    
    title(evalTitle);
    xlabel('Index of the sample (ordered by measured traversability cost)');
    ylabel('Traversability cost');
    legend({'Measured', 'Predicted'},'Location','EastOutside');
end

hold off;


