disp(['Loading raw data from ',dataDirectory,' ...']);

rollPitchYawSuffix = '_roll_pitch_yaw.csv';
motorCurrentSuffix = '_motor_currents.csv';
pointCloudSuffix = '_point_cloud.csv';
imageSuffix = '_image.jpg';

dirResult = dir(strcat(dataDirectory, '*', motorCurrentSuffix));
nbOfSamples = size(dirResult,1);

% Init dataset structure (slow to always append)
dataset = repmat(datasetStruct, nbOfSamples, 1);

nbOfSamples = length(dataset);
for i=1:nbOfSamples
    fileName = dirResult(i).name;
    [fileExt, filePrefix] = regexp(...
        fileName, motorCurrentSuffix, 'match','split');
    dataset(i).name  = filePrefix{1};
    
    disp(['Loading : ' dataset(i).name]);
    
    % Roll pitch yaw
    dataset(i).rollPitchYaw = csvread(strcat(...
        dataDirectory, dataset(i).name, rollPitchYawSuffix));
    
    % Currents
    dataset(i).rawCurrents = csvread(strcat(...
        dataDirectory, dataset(i).name, motorCurrentSuffix));
    
    % Point cloud
    startingRow = 1; % ignore header (zero based)
    startingCol = 0; % take all (zero based)
    dataset(i).rawPointCloud = csvread(strcat(...
        dataDirectory, dataset(i).name, pointCloudSuffix),...
        startingRow, startingCol);
    
    dataset(i).image = imread(strcat(...
        dataDirectory, dataset(i).name, imageSuffix));
end

% Shuffle the data to prevent any pattern from filenames or acquisition
% order
dataset = dataset(randperm(length(dataset)));
