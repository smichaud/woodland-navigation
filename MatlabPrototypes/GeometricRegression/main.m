% This is the main script, run it and answer the questions !

if ~exist('dataset','var') ... % If there is no dataset, need a reset
        || strcmp(questdlg('Reset all ?', '','Yes','No','No'),'Yes')
    % ========== Init/Reset ===================================================
    clear all;
    close all;
    clc;
    addpath(genpath('.'));
    
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
        'groundThreshold', 0.12); % Lower than half wheel and robot body
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
    regressionInfo = struct(...
        'featureNames', [],...
        'features', [],...
        'labels', [],...
        'nbOfTrees', 500,...
        'nbOfLeaves', 5);
end

rng(1987,'twister'); % Seed for random number generator

if isempty(dataset) ||...
    strcmp(questdlg('Reload data ?', '','Yes','No','No'),'Yes')
    if exist(strcat(dataDirectory, 'data.mat'), 'file') == 2 && ...
            strcmp(questdlg('Load from .mat ?', '','Yes','No','Yes'),'Yes')
        disp(['Loading data from ',datasetName, ' ...']);
        loadSavedData;
    else
        disp(['Loading raw data from ',dataDirectory,' ...']);
        loadRawData;
    end
end

% Do it all the time, it's not so long anyway
extractTraversabilityCost; % add the label to the structure
extractAreaOfInterest; % extract the 3D area to be traversed by the robot
extractAllFeatures;

if strcmp(questdlg('Show all processed samples ?', '','Yes','No','No'),...
        'Yes')
    showAllData;
end

convertDataForRegressor;
if strcmp(questdlg('Analyse data ?', '','Yes','No','Yes'),'Yes')
    findLeafSize;
    estimateFeatureImportance;
    %     bestFeaturesResults;
    %     findOutliers;
end

% if wantToTestLeaveOneOut
%     testLeaveOneOut;
% end
% 
% if wantToTrainRegressor
%     trainRegressor;
% end
% 
% % regressor.predict(sample); % Traversability cost prediction

if strcmp(questdlg('Show results ?', '','Yes','No','Yes'),'Yes')
    showResults;
end

if strcmp(questdlg('Save data to file ?', '','Yes','No','No'),'Yes')
    saveData;
end

% clearUselessVariables;
