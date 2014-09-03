clear all;
close all;
clc;

dataDirectory = '~/Desktop/ExtractedData/';
motorCurrentSuffix = '_motor_currents.csv';
pointCloudSuffix = '_point_cloud.csv';
imageSuffix = '.jpg';

dirResult = dir(strcat(dataDirectory, '*', motorCurrentSuffix));
nbOfSamples = size(dirResult,1);

% Init dataset structure (slow to always append)
dataset(nbOfSamples) = struct(...
    'name', '',...
    'rawCurrents', [],...
    'traversabilityCost', [],...
    'rawPointCloud', [],...
    'regionOfInterest', [],...
    'image', []);

approxCurrentStopped = 0.5900;
approxCurrentStartPeak = 6.0;

for i=1:nbOfSamples
    fileName = dirResult(i).name;
    [fileExt, filePrefix] = regexp(...
        fileName, motorCurrentSuffix, 'match','split');
    dataset(i).name  = filePrefix{1};
    
    disp(strcat('Processing: ', fileName));
    
    dataset(i).rawCurrents = csvread(strcat(...
        dataDirectory, dataset(i).name, motorCurrentSuffix));
    
    startingRow = 1; % ignore header (zero based)
    startingCol = 0; % take all (zero based)
    dataset(i).rawPointCloud = csvread(strcat(...
        dataDirectory, dataset(i).name, pointCloudSuffix),...
        startingRow, startingCol);
    
    dataset(i).rawPointCloud = imread(strcat(...
        dataDirectory, dataset(i).name, imageSuffix));
    
%     clf;
%     hold on;
%     plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,2), 'b');
%     plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,3), 'r');
%     uiwait;
end
