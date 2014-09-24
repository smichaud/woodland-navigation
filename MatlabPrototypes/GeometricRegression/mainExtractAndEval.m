tic
evaluations = [];
nbOfSamples = length(dataset);
for i=1:nbOfSamples
    dataset(i).features = containers.Map;
end

extractDensity;
extractMeanPoint;
extractEigen;

extractLayersXZ;
extractLayersXY;
extractLayersYZ;
% extractColumnZ;

extractHistogramXZ;
extractHistogramZ;
extractHistogramVoxels;

prepareDataForRegression;
findLeafSize
estimateFeatureImportance

% evalMeanAsPrediction

% evalRobustFitDensity;
% evalRobustFitAllFeatures;

% evalRandomForestDensity;
evalRandomForestAllFeatures;

runTimeInMinutes = toc/60

showEvaluations2;
uiwait;
showFeaturesImportanceEstimation;
