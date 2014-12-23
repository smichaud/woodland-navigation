% This is the main script, run it and answer the questions !
tic

% Set the 'justRunAll' to true to run all the script (unsupervised run)
justRunAll = false;

rng(1987,'twister'); % Seed for random number generator

if justRunAll || ~exist('dataset','var') ...
        || strcmp(questdlg('Reset all ?', '','Yes','No','No'),'Yes')
    initScript;
end

if justRunAll || isempty(dataset) ||...
        strcmp(questdlg('Reload dataset ?', '','Yes','No','No'),'Yes')
    if ~justRunAll &&...
            exist(strcat(dataDirectory, 'data.mat'), 'file') == 2 && ...
            strcmp(questdlg('Load from .mat ?', '','Yes','No','Yes'),'Yes')
        loadSavedData;
    else
        loadRawData;
    end
end

extractAreaOfInterest; % point cloud area of interest
extractTraversabilityCost; % add the label to the structure
computeImuDft;
computeCurrentsDft;
extractAllFeatures;

if ~justRunAll && strcmp(questdlg('Show all processed samples ?',...
        '','Yes','No','No'),'Yes')
    showDataset;
end

prepareDataForRegression;
if justRunAll || ...
        strcmp(questdlg('Analyse data ?', '','Yes','No','Yes'),'Yes')
    findLeafSize;
    estimateFeatureImportance;
    % findOutliers;
end

evaluations = [];
if justRunAll || ...
        strcmp(questdlg('Do the leave-one-out evaluation ?', '',...
        'Yes','No','No'),'Yes')
    evalMeanAsPrediction;
    evalTotalMeanAsPrediction;
    %     evalMedianAsPrediction;
    %     evalRandomAsPrediction;
    
    evalRobustFitDensity;
    %     evalRobustFitAllFeatures;
    
    %     evalRandomForestDensity;
    evalRandomForestBestFeatures;
    evalRandomForestAllFeatures;
end

if ~justRunAll && ...
        strcmp(questdlg('Show evaluations ?', '','Yes','No','Yes'),'Yes')
    showFeaturesImportanceEstimation;
    showEvaluations;
end

runTimeInMinutes = toc/60 % put it here to be saved

if justRunAll || ...
        strcmp(questdlg('Save data to file ?', '','Yes','No','No'),'Yes')
    saveData;
end

% clearUselessVariables;


