% This is the main script for traversability cost regression using
% geometric features. Adjust it in the "Set parameters" section

inputdlg('Prompt','Title', 1, {'20'})

% ========== Init =========================================================
clear all;
close all;
clc;
addpath(genpath('.')); % genpath is the workaround for recursive addpath

% ========== Define global variables and data structures ==================
dataDirectory = '../Data/';
datasetName = 'data.mat';
regressorName = 'regressor.mat';

dataset = []; % Variable to contain the dataset
regressor = []; % Variable to contain the regressor

robotSpeed = 0.3; % m/s
areaOfInterest = struct(...
    'distFromRobot', 2.0,...
    'depth', 1.6,...
    'width', 0.8,...
    'height', 1.5,...
    'groundThreshold', 0.12); % Lower than half wheel and robot body: 12 cm
datasetStruct = struct(...
    'name', '',...
    'rawCurrents', [],...
    'traversabilityStartIndex', [],...
    'traversabilityStopIndex', [],...
    'traversabilityCost', [],...
    'rawPointCloud', [],...
    'groundHeight', [],...
    'areaOfInterest', [],...
    'image', [],...
    'features', containers.Map);

% ========== Set parameters ===============================================
wantToLoadSavedData = true; % If it exist, of course
wantToShowData = false; % Image, point cloud area of interest, currents/cost
wantToShowResults = false; % Feature/cost, regression
wantToAnalyseData = true; % To check data/regressor information
wantToTestLeaveOneOut = false;
wantToTrainRegressor = false; % Usually data analysis done first
wantToSaveData = true; % At the end

% ========== Processing part ! ============================================
if exist(strcat(dataDirectory, 'data.mat'), 'file') == 2 ...
        && wantToLoadSavedData;
    disp('Dataset file exist and will be loaded...')
    loadSavedData;
else
    disp('Raw data will be loaded and processed...');
    loadRawData; % stock raw data from a folder in the structure "dataset"
    extractTraversabilityCost; % add the label to the structure
    extractAreaOfInterest; % extract the 3D area to be traversed by the robot
    extractAllFeatures;
end

initRegression;
if wantToAnalyseData
    findLeafSize;
    estimateFeatureImportance;
%     bestFeaturesResults;
%     findOutliers;
end

if wantToTestLeaveOneOut
    testLeaveOneOut;
end

if wantToTrainRegressor
    trainRegressor;
end

% regressor.predict(sample); % Traversability cost prediction

if wantToShowData
    showData; 
end

if wantToShowResults
    showResults; 
end

if wantToSaveData
    saveData;
end

clearUselessVariables;
