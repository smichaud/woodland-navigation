% Reference for my regression process :
% http://www.mathworks.com/help/stats/ensemble-methods.html#bsx62vu

rng(1987,'twister'); % Seed for random number generator
regressorFeatures = []; % Constant name for features
regressorLabels = []; % Constant name for Labels

convertDataForRegressor;