function [meanR2 r2PerLabel]  = ...
    rSquaredMultiLabels(realLabels, predictions)
% Return the R^2 metric for 2 vector (real labels and predicted labels)
meanValue = mean(realLabels);
nbOfValues = size(realLabels,1);
meanVector = repmat(meanValue, nbOfValues, 1);

SSE = sum(((realLabels - predictions).^2),1);
SST = sum(((realLabels - meanVector).^2),1);

r2PerLabel = 1 - (SSE./SST);
meanR2 = mean(r2PerLabel);
end