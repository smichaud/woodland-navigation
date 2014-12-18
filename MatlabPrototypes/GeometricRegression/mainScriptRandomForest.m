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
testResultStruct = struct(...
    'datasetReordering', [],...
    'validationResults', [],...
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

nbOfTest = 2; % 3;
nbOfTreesToTry = [10 50 100 150 200 250];
nbOfLeavesToTry = [1 5 10 20];
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
    
    nbOfTries = length(nbOfTreesToTry)*length(nbOfLeavesToTry);
    validationResults = repmat(validationResultStruct, nbOfTries,1);
    
    % ===== Hyperparameters search
    recordedTime = [];
    bestValidationRSquared = -inf;
    bestValidationIndex = [];
    bestValidationFeaturesIndexes = [];
    resultIndex = 1;
    for nbOfTrees = nbOfTreesToTry
        for nbOfLeaves = nbOfLeavesToTry
            validationResults(resultIndex).nbOfTrees = nbOfTrees;
            validationResults(resultIndex).nbOfLeaves = nbOfLeaves;
            validationResults(resultIndex).keptFeatureNames = ...
                cell(nbOfFeaturesToKeep);
            validationResults(resultIndex).rSquared = ...
                zeros(nbOfFeaturesToKeep, 1);
            
            % ===== Greedy features selection
            keptFeatureIndex = 0;
            keptFeatures = [];
            while keptFeatureIndex < nbOfFeaturesToKeep
                keptFeatureIndex = keptFeatureIndex + 1;
                
                tic
                disp(sprintf('Hyperparams(%d of %d):feature(%d of %d)',...
                    resultIndex, nbOfTries,...
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
                validationResults(resultIndex).keptFeatureNames{keptFeatureIndex} = ...
                    featureNames(newBestFeature);
                validationResults(resultIndex).rSquared(keptFeatureIndex) =...
                    bestRSquared;
                
                recordedTime(end+1) = toc;
                
                if bestRSquared > bestValidationRSquared
                    bestValidationRSquared = bestRSquared;
                    bestValidationIndex = resultIndex;
                    bestValidationFeaturesIndexes = keptFeatures;
                end
            end
            resultIndex = resultIndex+1;
        end
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
