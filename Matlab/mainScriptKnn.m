initScript;
disp('Start the magic !');

% =============== Init data
disp('Loading data...');
load([dataDirectory 'dataset.mat']);
% loadRawData;

computeImuDft;
computeCurrentsDft;
extractTraversabilityCost; % add the label to the structure
extractAreaOfInterest; % point cloud area of interest

% =============== Point cloud features extraction
resetFeatures;

%extractDensity;
%extractHighestPoint;
%extractMeanPoint;
%extractEigen;

%extractLayersXZ;
%extractLayersXY;
%extractLayersYZ;
%extractColumnZ;

%extractHistogramXZ;
%extractHistogramZ;
%extractHistogramVoxels;
extractVoxelMap;

% =============== Result structure
testResultStruct = struct(...
    'datasetReordering', [],...
    'dftParams', imuDftParams,...
    'minkowskiDistance', 1,...
    'validationResults', [],...
    'recordedTime', [],...
    'rSquared', []);

validationResultStruct = struct(...
    'nbOfKnnUsed', [],...
    'keptFeatureNames', [],... % Ordered by the greedy selection
    'rSquared', [],... % Vector: for 1 feature, for 2 features...
    'rSquaredPerLabel', []); % Matrix, each line == 1 feature, 2 feat 

% =============== Machine learning part !
% ===== Init
nbOfSamples = length(dataset);
nbOfFeatures = length(cell2mat(dataset(1).features.values));

[featuresOrdered labelsOrdered featureNames] = ...
    createDatasetMatrices(dataset);
imuDftVectorsOrdered = getImuDftVectors(dataset, imuDftParams);

trainingSetSize = 0.75;
testSetSize = 1-trainingSetSize;

nbOfTest = 3;
nbOfKnnToTry = [1 2 3 4 5 6 7 8 9 10];
nbOfFeaturesToKeep = nbOfFeatures; % nbOfFeatures to keep all

testResults = repmat(testResultStruct, nbOfTest,1);
for testIndex = 1:nbOfTest
    reordering = randperm(nbOfSamples);
    testResults(testIndex).datasetReordering = reordering;
    
    features = featuresOrdered(reordering,:);
    imuDftVectors = imuDftVectorsOrdered(reordering,:);
    
    trainIndexes = 1:round(nbOfSamples*trainingSetSize);
    testIndexes = round(nbOfSamples*trainingSetSize+1):nbOfSamples;
    trainFeatures = features(trainIndexes,:);
    testFeatures = features(testIndexes,:);
    trainImuDftVectors = imuDftVectors(trainIndexes,:);
    testImuDftVectors = imuDftVectors(testIndexes,:);
    
    nbOfTries = length(nbOfKnnToTry);
    validationResults = repmat(validationResultStruct, nbOfTries,1);
    
    % ===== Hyperparameters search
    recordedTime = [];
    bestValidationRSquared = -inf;
    bestValidationIndex = [];
    bestValidationFeaturesIndexes = [];
    validationIndex = 1;
    for nbOfKnnUsed = nbOfKnnToTry
        validationResults(validationIndex).nbOfKnnUsed = nbOfKnnUsed;
        validationResults(validationIndex).keptFeatureNames = ...
            cell(nbOfFeaturesToKeep);
        validationResults(validationIndex).rSquared = [];
        validationResults(validationIndex).rSquaredPerLabel = [];        
        
        % ===== Greedy features selection
        keptFeatureIndex = 0;
        keptFeatures = [];
        while keptFeatureIndex < nbOfFeaturesToKeep
            keptFeatureIndex = keptFeatureIndex + 1;
            
            tic
            disp(sprintf('KNN(%d of %d):feature(%d of %d)',...
                validationIndex, nbOfTries,...
                keptFeatureIndex, nbOfFeaturesToKeep));
            
            featuresToTry = 1:nbOfFeatures;
            featuresToTry(keptFeatures) = [];
            
            bestFeatureRSquared = -inf;
            bestRSquaredPerLabel = [];
            newBestFeature = [];
            for currentFeature = featuresToTry
                currentFeaturesVector = [keptFeatures currentFeature];

                [rSquaredResult, rSquaredResultPerLabel] = ...
                    knnLeaveOneOut(nbOfKnnUsed,...
                    testResults(testIndex).minkowskiDistance,...
                    trainFeatures(:,currentFeaturesVector),...
                    trainImuDftVectors);
                
                if rSquaredResult > bestFeatureRSquared
                    newBestFeature = currentFeature;
                    bestFeatureRSquared = rSquaredResult;
                    bestRSquaredPerLabel = rSquaredResultPerLabel;
                end
            end
            keptFeatures(keptFeatureIndex) = newBestFeature;
            validationResults(validationIndex).keptFeatureNames{keptFeatureIndex} = ...
                featureNames(newBestFeature);
            validationResults(validationIndex).rSquared(end+1) = ...
                bestFeatureRSquared;
            validationResults(validationIndex).rSquaredPerLabel(end+1,:) = ...
                bestRSquaredPerLabel;
            
            recordedTime(end+1) = toc;
            
            if bestFeatureRSquared > bestValidationRSquared
                bestValidationRSquared = bestFeatureRSquared;
                bestValidationIndex = validationIndex;
                bestValidationFeaturesIndexes = keptFeatures;
            end
        % ===== Greedy features selection (END)
        end
        validationIndex = validationIndex+1;
    end
    
    testResults(testIndex).validationResults = validationResults;
    testResults(testIndex).recordedTime = recordedTime;
    
    % ===== Test the final regressor    
    testLabelPredictions = knnRegression(...
        trainFeatures(:,bestValidationFeaturesIndexes),...
        trainImuDftVectors,...
        testFeatures(:,bestValidationFeaturesIndexes),...
        validationResults(bestValidationIndex).nbOfKnnUsed,...
        testResults(testIndex).minkowskiDistance);

    testResults(testIndex).rSquared =...
        rSquared(testLabelPredictions, testImuDftVectors)
    
    % =============== Saving (for each test just in case)
    saveName = [datestr(now,'yyyy_mm_dd_HH_MM_SS')...
        '_KnnResults_test_'...
        int2str(testIndex) 'of' int2str(nbOfTest) '.mat'];
    save(['./results/', saveName],...
        'areaOfInterest', 'featuresOrdered', 'imuDftVectorsOrdered',...
        'featureNames', 'trainingSetSize', 'testResults');
end
