initLabeling;

displayConfiguration = [4 2];
nbOfSamplePerFigure = prod(displayConfiguration);
vectorSize = size(dftVectors,2);

featureNorms = zeros(nbOfSamples,1);
for i = 1:nbOfSamples
    featureNorms(i) = norm(dftVectors(i,:));
%     featureNorms(i) = sum(dftVectors(i,:));
end

[sortedValues,sortedIndexes] = sort(featureNorms);

for i = 1:nbOfSamplePerFigure:nbOfSamples
    figure('Name', 'Data features vectors',...
        'units','normalized', 'outerposition',[0 0 1 1]);
    
    subI = 0;
    while (i+subI <= nbOfSamples && subI < nbOfSamplePerFigure)
        subI = subI + 1;
        subplot(displayConfiguration(1),displayConfiguration(2),subI);
        plot(1:vectorSize, dftVectors(sortedIndexes(i-1+subI),:));
%         set(gca, 'ylim', [0 .1]);
        title(sprintf('Sample : %d', i-1+subI));
    end
    
    waitforbuttonpress;
    close all;
end