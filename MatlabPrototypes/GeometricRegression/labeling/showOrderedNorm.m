initLabeling;

imageConfiguration = [2 3];
nbOfImagePerFigure = prod(imageConfiguration);

featureNorms = zeros(nbOfSamples,1);
for i = 1:nbOfSamples
%     featureNorms(i) = norm(dftVectors(i,:));
    featureNorms(i) = sum(dftVectors(i,:));
end

[sortedValues,sortedIndexes] = sort(featureNorms);

for i = 1:nbOfImagePerFigure:nbOfSamples
    figure('Name', 'Data sorted by distance from the origin',...
        'units','normalized', 'outerposition',[0 0 1 1]);
    
    subI = 0;
    while (i+subI <= nbOfSamples && subI < nbOfImagePerFigure)
        subI = subI + 1;
        subplot(imageConfiguration(1),imageConfiguration(2),subI);
        imshow(dataset(sortedIndexes(i-1+subI)).image);
        title(sprintf('Order : %d', i-1+subI));
    end
    
    waitforbuttonpress;
    close all;
end