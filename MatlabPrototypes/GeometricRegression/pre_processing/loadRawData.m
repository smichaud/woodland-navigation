disp(['Loading raw data from ',dataDirectory,' ...']);

rollPitchYawSuffix = '_roll_pitch_yaw.csv';
motorCurrentSuffix = '_motor_currents.csv';
intertialMeasurementsSuffix = '_inertial_measurements.csv';
pointCloudSuffix = '_point_cloud_0.csv';
icpOdometrySuffix = '_complete_transfo.txt';
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
    
    disp(['Loading : ' filePrefix{1}]);
    
    tempCurrents = csvread(strcat(...
        dataDirectory, filePrefix{1}, motorCurrentSuffix));
    
    dataset(i).name  = filePrefix{1};
    
    % Currents
    dataset(i).rawCurrents = csvread(strcat(...
        dataDirectory, dataset(i).name, motorCurrentSuffix));
    
    % IMU
    dataset(i).rawIMU = csvread(strcat(...
        dataDirectory, dataset(i).name, intertialMeasurementsSuffix));
    
    % Roll pitch yaw
    dataset(i).rollPitchYaw = csvread(strcat(...
        dataDirectory, dataset(i).name, rollPitchYawSuffix));
    
    % ICP odometry matrices
    dataset(i).icpOdometry = dlmread(strcat(...
        dataDirectory, dataset(i).name, icpOdometrySuffix));
    
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
