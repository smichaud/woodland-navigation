initScript;

% =============== Init data
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
resultStruct = struct(...
        'nbOfTrees', [],...
        'nbOfLeaves', [],...
        'keptFeatureNames', [],... % Ordered by the greedy selection
        'validationRSquared', []); % For 1 feature, 2 features...

% =============== Machine learning part !
% ===== Init
nbOfSamples = length(dataset);
nbOfFeatures = length(cell2mat(dataset(1).features.values));

datasetReordering = randperm(length(dataset));
dataset = dataset(datasetReordering); % Shuffle dataset
[features labels featureNames] = createDatasetMatrices(dataset);

trainingSetSize = 0.75;
testSetSize = 1-trainingSetSize;
trainFeatures = features(1:round(nbOfSamples*trainingSetSize), :);
trainLabels = labels(1:round(nbOfSamples*trainingSetSize), :);
testFeatures = features((round(nbOfSamples*trainingSetSize)+1):end, :);
testLabels = features((round(nbOfSamples*trainingSetSize)+1):end, :);

nbOfTreesToTry = [10 50 100 150 200 250];
nbOfLeavesToTry = [1 5 10 20 50];
nbOfFeaturesToKeep = nbOfFeatures; % nbOfFeatures to keep all

results = repmat(resultStruct,...
    length(nbOfTreesToTry)*length(nbOfLeavesToTry),1);

% ===== Hyperparameters search
overallBestRSquared = -inf
overallBestIndex = [];
overallFeaturesIndexes = [];
resultIndex = 1;
for nbOfTrees = nbOfTreesToTry
    for nbOfLeaves = nbOfLeavesToTry        
        results(resultIndex).nbOfTrees = nbOfTrees;
        results(resultIndex).nbOfLeaves = nbOfLeaves;
        results(resultIndex).keptFeatureNames = cell(nbOfFeaturesToKeep);
        results(resultIndex).validationRSquared = ...
            zeros(nbOfFeaturesToKeep, 1);
        
        % ===== Greedy features selection
        keptFeatures = [];
        while length(keptFeatures) < nbOfFeaturesToKeep
            featuresToTry = 1:nbOfFeatures;
            featuresToTry(keptFeatures) = [];
            
            bestRSquared = -inf;
            newBestFeature = [];
            for currentFeature = featuresToTry
                currentFeaturesVector = [keptFeatures currentFeature];
                
                rSquaredResult = randomForestleaveOneOut(...
                    nbOfTrees, nbOfLeaves,...
                    trainFeatures(:,currentFeaturesVector), trainLabels);
                
                if rSquaredResult > bestRSquared
                    newBestFeature = currentFeature;
                    bestRSquared = rSquaredResult;
                end
            end
            keptFeatures(end+1) = newBestFeature;
            results(resultIndex).keptFeatureNames{end+1} = ...
                featureNames(newBestFeature);
            results(resultIndex).validationRSquared(end+1,1) =...
                bestRSquared;
            
            if bestRSquared > overallBestRSquared
                overallBestRSquared = bestRSquared;
                overallBestIndex = resultIndex;
                overallFeaturesIndexes = keptFeatures;
            end
        end
        resultIndex = resultIndex+1;
    end
end

% ===== Test the final regressor
testRegressor = TreeBagger(results(overallBestIndex).nbOfTrees,...
    trainFeatures(:, overallFeaturesIndexes), trainLabels,...
    'Method', 'R', 'MinLeaf', results(overallBestIndex).nbOfLeaves);

nbOfSamplesTest = length(testLabels)
testLabelPredictions = zeros(nbOfSamplesTest,1);
for i = 1:nbOfSamplesTest    
    testLabelPredictions(i) = testRegressor.predict(...
        testFeatures(i,overallFeaturesIndexes));
end
testRSquared = rSquared(testLabelPredictions, testLabels)

% =============== Saving
saveName = [datestr(now,'yyyy_mm_dd_HH_MM_SS') '_randomForestResults.mat'];
save(['./results/', saveName]),...
    'areaOfInterest', 'traversabilityCostInfo', 'datasetReordering',...
    'features', 'labels', 'featureNames', 'trainingSetSize',...    
    'results', 'testRSquared'); 
% Having the point cloud features param could be nice
