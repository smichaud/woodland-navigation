motorCurrentSuffix = '_motor_currents.csv';
pointCloudSuffix = '_point_cloud.csv';
imageSuffix = '.jpg';

dirResult = dir(strcat(dataDirectory, '*', motorCurrentSuffix));
nbOfSamples = size(dirResult,1);

% Init dataset structure (slow to always append)
dataset = repmat(datasetStruct, nbOfSamples, 1);

for i=1:nbOfSamples
    fileName = dirResult(i).name;
    [fileExt, filePrefix] = regexp(...
        fileName, motorCurrentSuffix, 'match','split');
    dataset(i).name  = filePrefix{1};
    
    disp(strcat('Loading : ', dataset(i).name));
    
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
