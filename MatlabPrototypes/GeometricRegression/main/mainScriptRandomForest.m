initScript;

% =============== Init data
% loadSavedData;
loadRawData;

extractTraversabilityCost; % add the label to the structure
extractAreaOfInterest; % point cloud area of interest
computeImuDft;
computeCurrentsDft;

% =============== Point cloud features
resetFeatures;

extractDensity;
% extractHighestPoint;
% extractMeanPoint;
% extractEigen;

extractLayersXZ;
extractLayersXY;
extractLayersYZ;
extractColumnZ;

% extractHistogramXZ;
% extractHistogramZ;
% extractHistogramVoxels;
% extractVoxelMap;

% =============== Result structure
results = [];
resultStruct = struct(...
        'nbOfTrees', [],...
        'nbOfLeaves', [],...
        'keptFeatureNames', {});

% =============== Machine learning part !
% Create the features and labels matrices
nbOfSamples = length(dataset);
nbOfFeatures = 0;

nbOfTreesToTry = [50 100 150 200 250]
nbOfLeavesToTry = [5 10 20 50 100];

% Hyperparameters search
for nbOfTrees = nbOfTreesToTry
    for nbOfLeavesToTry = nbOfLeavesToTry
        % Greedy feature selection
        keptFeatures = [];
        while length(keptFeatures) <         
            for currentFeature = 1:nbOfFeatures

            end
        end
        % Save result
    end
end


% =============== Saving 
save(strcat(dataDirectory,'workspace')); % Save all
save(strcat(dataDirectory,'results')); % Save results