clc;
clearvars -except 'justRunAll';
close all;
clc;
addpath(genpath('.'));
addpath(genpath('../Utils/'));

% ========== Define global variables and data structures ==============
dataDirectory = '../../../WoodlandNavigationData/RegressionV2/';
datasetName = 'data.mat';
%     slopeCorrectionFile = '../Data/SlopeCorrection/slopeCorrection.mat';
%     load(slopeCorrectionFile);

dataset = []; % Variable to contain the dataset
regressor = []; % Variable to contain the regressor

robotSpeed = 0.3; % m/s
areaOfInterest = struct(...
    'distFromRobot', 1.0,...
    'depth', 2.6,...
    'width', 1.0,...
    'height', 1.4,...
    'xTfAdjustment', 0.00,... % base_link + adjust = body front
    'groundThreshold', 0.12); % Lower than half wheel and robot body

traversabilityCostInfo = struct(...
    'wantToCorrectSlope', false,...
    'motorCurrentsIntegralMetric', 1,...
    'motorCurrentsVarianceMetric', 2,...
    'inertiaVarianceMetric', 3,...
    'odometryErrorMetric', 4,...
    'randomValueMetric', 5,...
    'traversabilityMetrics', 4);

datasetStruct = struct(...
    'name', '',...
    'rawCurrents', [],...
    'dftCurrents', [],...
    'rawIMU', [],...
    'dftIMU', [],...
    'rollPitchYaw', [],...
    'icpOdometry', [],...
    'traversabilityStartTime', [],...
    'traversabilityStopTime', [],...
    'traversabilityCost', [],...
    'userTraversabilityCost', [],...
    'rawPointCloud', [],...
    'groundHeight', [],...
    'areaOfInterest', [],...
    'image', [],...
    'features', containers.Map);
regressionInfo = struct(...
    'featureNames', [],...
    'featuresImportance', [],...
    'trainingFeatures', [],...
    'trainingLabels', [],...
    'testFeatures', [],...
    'testLabels', [],...
    'nbOfTrees', 500,...
    'nbOfLeaves', 5);
evaluationStruct = struct(...
    'name', [],...
    'labels', [],...
    'meanSquaredError', [],...
    'rSquared', []);
evaluations = []; % To store all leave one out results

testRegressor = [];