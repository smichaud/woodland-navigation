function showResultsProgress(results)

for i = 0:5:length(results)
    markerList = {'or-', '+g-', '*b-', 'xm-', 'sk-'};
    legendNames = cell(0);
    figure;
    hold on;
    for j = 1:5
        k = i+j;
        if k <= length(results)
        plot(1:length(results(k).rSquared),...
            results(k).rSquared,...
            markerList{j});
        legendNames{j} = sprintf('%d Trees and %d Leaves',...
            results(k).nbOfTrees, results(k).nbOfLeaves);
        end
    end
    legend(legendNames, 'Location', 'EastOutside');
end

end

