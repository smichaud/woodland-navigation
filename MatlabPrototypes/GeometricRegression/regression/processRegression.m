% Reference :
% http://www.mathworks.com/help/stats/ensemble-methods.html#bsx62vu

rng(1987,'twister');
prepareData;
% findLeafSize;
% estimateFeatureImportance;
% bestFeaturesResults;
% findOutliers;

nbOfTrees = 500;
nbOfLeaves = 5;

trainRegressor;

% ========== Prediction
% regressor.predict(sample);



