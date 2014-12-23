initScript;
disp('Start the magic !');

% =============== Init data
disp('Loading data...');
load([dataDirectory 'dataset.mat']);
% loadRawData;

extractTraversabilityCost; % add the label to the structure
extractAreaOfInterest; % point cloud area of interest
computeImuDft;
computeCurrentsDft;

% =============== Point cloud features extraction
resetFeatures;

extractDensity;
extractHighestPoint;
extractMeanPoint;
extractEigen;

extractLayersXZ;
extractLayersXY;
extractLayersYZ;
extractColumnZ;

extractHistogramXZ;
extractHistogramZ;
extractHistogramVoxels;
% extractVoxelMap;

% =============== Result structure
featuresSelectStopStreak = 2; % If not improvement after X, stop the select
testResultStruct = struct(...
    'datasetReordering', [],...
    'validationResults', [],...
    'featuresSelectStopStreak', featuresSelectStopStreak,... 
    'recordedTime', [],...
    'rSquared', []);

validationResultStruct = struct(...
    'nbOfTrees', [],...
    'nbOfLeaves', [],...
    'keptFeatureNames', [],... % Ordered by the greedy selection
    'rSquared', []); % For 1 feature, 2 features...

% =============== Machine learning part !
% ===== Init
nbOfSamples = length(dataset);
nbOfFeatures = length(cell2mat(dataset(1).features.values));

[featuresOrdered labelsOrdered featureNames] = ...
    createDatasetMatrices(dataset);

trainingSetSize = 0.75;
testSetSize = 1-trainingSetSize;

nbOfTest = 3;
% create random tuple of nbOfTrees/nbOfLeaves
nbOfValidations = 25;
nbOfTreesRange = [10 250];
nbOfLeavesRange = [1 20];
validationsHyperparameters = ...
    [unidrnd(diff(nbOfTreesRange),nbOfValidations,1) + nbOfTreesRange(1) ,...
    unidrnd(diff(nbOfLeavesRange),nbOfValidations,1) + nbOfLeavesRange(1)];

nbOfFeaturesToKeep =  nbOfFeatures; % nbOfFeatures to keep all

testResults = repmat(testResultStruct, nbOfTest,1);
for testIndex = 1:nbOfTest
    testResults(testIndex).datasetReordering = randperm(nbOfSamples);
    features = featuresOrdered(testResults(testIndex).datasetReordering,:);
    labels = labelsOrdered(testResults(testIndex).datasetReordering, :);
    
    trainFeatures = features(1:round(nbOfSamples*trainingSetSize), :);
    trainLabels = labels(1:round(nbOfSamples*trainingSetSize), :);
    testFeatures = features((round(nbOfSamples*trainingSetSize)+1):end, :);
    testLabels = labels((round(nbOfSamples*trainingSetSize)+1):end, :);

    validationResults = repmat(validationResultStruct, nbOfValidations,1);
    
    % ===== Hyperparameters search
    recordedTime = [];
    bestValidationRSquared = -inf;
    bestValidationIndex = [];
    bestValidationFeaturesIndexes = [];
    validationIndex = 1;
    for hyperParams = 1:size(validationsHyperparameters,1)
        nbOfTrees = validationsHyperparameters(hyperParams,1);
        nbOfLeaves = validationsHyperparameters(hyperParams,2);
        
        validationResults(validationIndex).nbOfTrees = nbOfTrees;
        validationResults(validationIndex).nbOfLeaves = nbOfLeaves;
        validationResults(validationIndex).keptFeatureNames = ...
            cell(nbOfFeaturesToKeep);
        validationResults(validationIndex).rSquared = ...
            zeros(nbOfFeaturesToKeep, 1);
        
        % ===== Greedy features selection
        keptFeatureIndex = 0;
        keptFeatures = [];
        currentFeaturesStreak = 0;
        while keptFeatureIndex < nbOfFeaturesToKeep && ...
            currentFeaturesStreak < featuresSelectStopStreak
            keptFeatureIndex = keptFeatureIndex + 1;
            
            tic
            disp(sprintf('Hyperparams(%d of %d):feature(%d of %d)',...
                validationIndex, nbOfValidations,...
                keptFeatureIndex, nbOfFeaturesToKeep));
            
            featuresToTry = 1:nbOfFeatures;
            featuresToTry(keptFeatures) = [];
            
            bestRSquared = -inf;
            newBestFeature = [];
            for currentFeature = featuresToTry
                currentFeaturesVector = [keptFeatures currentFeature];
                
                rSquaredResult = randomForestLeaveOneOut(...
                    nbOfTrees, nbOfLeaves,...
                    trainFeatures(:,currentFeaturesVector),...
                    trainLabels);
                
                if rSquaredResult > bestRSquared
                    newBestFeature = currentFeature;
                    bestRSquared = rSquaredResult;
                end
            end
            keptFeatures(keptFeatureIndex) = newBestFeature;
            validationResults(validationIndex).keptFeatureNames{keptFeatureIndex} = ...
                featureNames(newBestFeature);
            validationResults(validationIndex).rSquared(keptFeatureIndex) =...
                bestRSquared;
            
            recordedTime(end+1) = toc;
            
            currentFeaturesStreak = currentFeaturesStreak + 1;
            
            if bestRSquared > bestValidationRSquared
                bestValidationRSquared = bestRSquared;
                bestValidationIndex = validationIndex;
                bestValidationFeaturesIndexes = keptFeatures;
                currentFeaturesStreak = 0;
            end
        end
        validationIndex = validationIndex+1;
    end
    
    testResults(testIndex).validationResults = validationResults;
    testResults(testIndex).recordedTime = recordedTime;
    
    % ===== Test the final regressor
    testRegressor = TreeBagger(...
        validationResults(bestValidationIndex).nbOfTrees,...
        trainFeatures(:, bestValidationFeaturesIndexes), trainLabels,...
        'Method', 'R', 'MinLeaf', ...
        validationResults(bestValidationIndex).nbOfLeaves);
    
    nbOfSamplesTest = length(testLabels);
    testLabelPredictions = zeros(nbOfSamplesTest,1);
    for i = 1:nbOfSamplesTest
        testLabelPredictions(i) = testRegressor.predict(...
            testFeatures(i,bestValidationFeaturesIndexes));
    end
    testResults(testIndex).rSquared =...
        rSquared(testLabelPredictions, testLabels)
    
    % =============== Saving (for each test just in case)
    saveName = [datestr(now,'yyyy_mm_dd_HH_MM_SS')...
        '_randomForestResults_test_'...
        int2str(testIndex) 'of' int2str(nbOfTest) '.mat'];
    save(['./results/', saveName],...
        'areaOfInterest', 'traversabilityCostInfo',...
        'featuresOrdered', 'labelsOrdered', 'featureNames',...
        'trainingSetSize', 'testResults');
end