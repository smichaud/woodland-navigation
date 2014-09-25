disp(['Loading raw data from ',dataDirectory,' ...']);

motorCurrentSuffix = '_motor_currents.csv';
rollPitchYawSuffix = '_roll_pitch_yaw.csv';

dirResult = dir(strcat(dataDirectory, '*', motorCurrentSuffix));
nbOfSamples = size(dirResult,1);

% Init dataset structure (slow to always append)
rawData = repmat(rawDataStruct, nbOfSamples, 1);

nbOfSamples = length(rawData);
for i=1:nbOfSamples
    fileName = dirResult(i).name;
    [fileExt, filePrefix] = regexp(...
        fileName, motorCurrentSuffix, 'match','split');
    rawData(i).name  = filePrefix{1};
    
    disp(['Loading : ' rawData(i).name]);

    rawData(i).rawMotorCurrents = csvread(strcat(...
        dataDirectory, rawData(i).name, motorCurrentSuffix));
    rawData(i).rawRollPitchYaw = csvread(strcat(...
        dataDirectory, rawData(i).name, rollPitchYawSuffix));
end
