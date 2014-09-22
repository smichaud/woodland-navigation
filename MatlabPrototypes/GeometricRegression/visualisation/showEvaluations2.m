% Start by showing 1 graph per result (real label vs prediction)
clf;
close all;
figure('Name', 'Evaluation figure', 'units','normalized',...
    'outerposition',[0 0 1 1])
hold on;

[vector orderedIndexes] = sort(regressionInfo.trainingLabels);
markerList = {'or', '+g', '*b', 'xm', 'sk', 'pg', 'db'};
evalLegend = {'Measured label'};

nbOfTrainingSamples = length(regressionInfo.trainingLabels);
plot(1:nbOfTrainingSamples, ...
    regressionInfo.trainingLabels(orderedIndexes), markerList{1})

nbOfEvaluations = length(evaluations);
for i = 1:nbOfEvaluations
    plot(1:nbOfTrainingSamples, ...
        evaluations(i).labels(orderedIndexes), markerList{i+1})
    evalLegend{i+1} = ['Predicted, using ' evaluations(i).name];
    evalLegend{i+1} = [evalLegend{i+1} ' (MSE = ' ];
    evalLegend{i+1} = [evalLegend{i+1} num2str(evaluations(i).meanSquaredError)];
    evalLegend{i+1} = [evalLegend{i+1} ')'];
end
title('Traversability cost results');
xlabel('Index of the sample (ordered by measured traversability cost)');
ylabel('Traversability cost');
legend(evalLegend,'Location','EastOutside');

hold off;


